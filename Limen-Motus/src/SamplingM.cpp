#include "Sampling_M.h"
#include <stdexcept>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <system_error>
#include <string>
using Point3D = Eigen::Vector3d;
using Point2D = Eigen::Vector2d;
using Vector2D = Eigen::Vector2d;
using VectorOfPoint3D = std::vector<Point3D>;
using VectorOfVector2D = std::vector<Vector2D>;
namespace plt =matplotlibcpp;
#include <Python.h>

struct Point3DCompare {

    bool operator()(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) const {

        if (lhs.x() != rhs.x()) return lhs.x() < rhs.x();
        if (lhs.y() != rhs.y()) return lhs.y() < rhs.y();
        return lhs.z() < rhs.z();
    }
};

SamplingResult::SamplingResult(size_t num_points, size_t num_wheels,
                               const std::vector<Eigen::Vector2d>& wheel_coords,
                               const std::vector<Eigen::Vector2d>& keypoint_coords)
    :keypoint_coords_body_(keypoint_coords),
    wheel_coords_body_(wheel_coords),
    num_wheels(num_wheels),
    num_points(num_points)
{

    hexahedron_end_points_.reserve(num_points);

    neighbor_g_.resize(num_points);
    for (auto& row : neighbor_g_) {
        row.resize(num_points);
    }

    double incre_ratio=HybridAStar::Constants::sample_ratio;
    Tmax= incre_ratio * (1.0f/HybridAStar::Constants::positionResolution)*(std::sqrt(2))
        /HybridAStar::Constants::V_max_vision;

    if(HybridAStar::Constants::fix_axil && HybridAStar::Constants::Algorithm_Framework_underothers=="fix_axil"){
        get_hyAstar_SamplingTable();
        get_neighbor_ordi_g();
    }else if(HybridAStar::Constants::Algorithm_Framework == "AWS_from_others"&& HybridAStar::Constants::Algorithm_Framework_underothers == "mode_shifting"){

        getkeyPoint();
        get_mode_shifting_SamplingTable();
        get_neighbor_g();

        if(HybridAStar::Constants::output_samples){
            output_samples_to_json();
        }
    }
    else{

        getkeyPoint();
        getAWSConstructSamplingTable();
        get_neighbor_g();

        if(HybridAStar::Constants::output_samples){
            output_samples_to_json();
        }
    }
}

bool has_two_zero_coords(const Point3D& p) {

    int zero_count = 0;
    if (p.x() == 0.0) zero_count++;
    if (p.y() == 0.0) zero_count++;
    if (p.z() == 0.0) zero_count++;
    return zero_count == 2;
}

void SamplingResult::get_hyAstar_SamplingTable(){

    ordinary_hybrid_end_points_.clear();

    double V_max = HybridAStar::Constants::V_max_vision;
    double w_max = HybridAStar::Constants::w_max_wheel;
    double xy_resolution = 2*1.0 / HybridAStar::Constants::positionResolution;
    double yaw_resolution = 2 * HybridAStar::Constants::deltaHeadingRad;
    double dt = Tmax;

    ordinary_hybrid_end_points_.push_back(Eigen::Vector3d(xy_resolution, 0, 0));

    ordinary_hybrid_end_points_.push_back(Eigen::Vector3d(-xy_resolution, 0, 0));

    ordinary_hybrid_end_points_.push_back(Eigen::Vector3d(0, 0, yaw_resolution));

    ordinary_hybrid_end_points_.push_back(Eigen::Vector3d(0, 0, -yaw_resolution));

}

void SamplingResult::getAWSConstructSamplingTable()
{
    double xy_resolution =1.0f/HybridAStar::Constants::positionResolution;
    double yaw_resolution=HybridAStar::Constants::deltaHeadingRad;
    int sample_density=HybridAStar::Constants::sample_density;

    if (sample_density <= 1) {
        sample_density = 2;
    }
    int num_segments = sample_density;
    int points_per_edge = num_segments + 1;

    double dx = xy_resolution / 2.0;
    double dy = xy_resolution / 2.0;
    double dz = yaw_resolution / 2.0;
    std::vector<Point3D> vertices = {
        Point3D(-dx, -dy, -dz), Point3D(dx, -dy, -dz), Point3D(dx, dy, -dz), Point3D(-dx, dy, -dz),
        Point3D(-dx, -dy, dz),  Point3D(dx, -dy, dz),  Point3D(dx, dy, dz),  Point3D(-dx, dy, dz)
    };

    std::vector<std::vector<int>> faces = {
        {0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4},
        {3, 7, 6, 2}, {0, 4, 7, 3}, {1, 2, 6, 5}
    };

    std::set<Point3D, Point3DCompare> unique_points;

    for (const auto& face : faces) {

        if (face.size() < 4) {
            std::cerr << "Warning: Face has insufficient vertices: " << face.size() << " (expected >= 4)" << std::endl;
            continue;
        }

        if (face[0] < 0 || face[0] >= (int)vertices.size() ||
            face[1] < 0 || face[1] >= (int)vertices.size() ||
            face[3] < 0 || face[3] >= (int)vertices.size()) {
            std::cerr << "Warning: Invalid face indices: [" << face[0] << "," << face[1] << "," << face[3]
                      << "] for vertices.size() = " << vertices.size() << std::endl;
            continue;
        }

        const Point3D& p0 = vertices[face[0]];
        const Point3D& p1 = vertices[face[1]];
        const Point3D& p3 = vertices[face[3]];
        Point3D u_axis = p1 - p0;
        Point3D v_axis = p3 - p0;

        for (int i = 0; i < points_per_edge; ++i) {
            for (int j = 0; j < points_per_edge; ++j) {
                double u = static_cast<double>(i) / num_segments;
                double v = static_cast<double>(j) / num_segments;
                Point3D sampled_point = p0 + u * u_axis + v * v_axis;

                unique_points.insert(sampled_point);
            }
        }
    }
    hexahedron_end_points_=std::vector<Point3D>(unique_points.begin(), unique_points.end());

    std::sort(hexahedron_end_points_.begin(), hexahedron_end_points_.end(),

        [](const Point3D& p1, const Point3D& p2) {
            bool p1_has_two_zeros = has_two_zero_coords(p1);
            bool p2_has_two_zeros = has_two_zero_coords(p2);

            if (p1_has_two_zeros && !p2_has_two_zeros) {
                return true;
            }

            return false;
        }
    );

    double R=0.0;
    Point2D time_interval_end_point_p;
    Point3D time_interval_end_point;
    Point2D p_danger;
    Eigen::Vector2d ICM;

    bool find_omin=0;
    int photo_num=0;
    plt::figure();
    plt::xlim(-1, 1);
    plt::ylim(-1,1);
    for (const auto& p : hexahedron_end_points_) {
        double x = p[0];
        double y = p[1];
        double angle_rad = p[2];
        if(angle_rad>M_PI)angle_rad=angle_rad-M_PI*2.0f;
        Vector2D original_vector(x, y);
        PathState pathstate;
        if(angle_rad!=0){
            int8_t f = (angle_rad > 0) ? 1 : -1;

            Eigen::Matrix2d M_for_icm;
                M_for_icm << cos(angle_rad)-1, sin(angle_rad),
                    -1.0*sin(angle_rad), cos(angle_rad)-1;

            ICM = 1.0/(2*cos(angle_rad)-2) * (M_for_icm * original_vector) ;
            ICM_body_.push_back(ICM);

            R = ICM.norm();

            auto max_it = std::max_element(keypoint_coords_body_.begin(), keypoint_coords_body_.end(),
                [ICM](const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
                    return (p1 - ICM).squaredNorm() < (p2 - ICM).squaredNorm();
                }
            );
            p_danger=*max_it;
            point_danger.push_back(p_danger);
            double womega = HybridAStar::Constants::V_max_vision/(*max_it - ICM).norm();
            double theta = womega * Tmax;

            Eigen::Matrix2d M_for_newp;
            M_for_newp << cos(angle_rad)-1, -1.0*sin(angle_rad),
                sin(angle_rad), cos(angle_rad)-1;

            time_interval_end_point_p=-1.0* (M_for_newp * ICM) ;
            time_interval_end_points_.push_back(
                Eigen::Vector3d(
                    time_interval_end_point_p[0],
                    time_interval_end_point_p[1],
                    static_cast<double>(theta * f)
                ));

            double norm_value = ICM.norm();
            V_max_.push_back(womega*norm_value);

            if(find_omin==0 && time_interval_end_point_p[0]==0 && time_interval_end_point_p[1]==0){
                omni_w_max = womega;
                find_omin=1;
            }
            Eigen::Rotation2D<double> rotation3(-M_PI_2*f);
            for (const auto& wheel_coord : wheel_coords_body_){

                Eigen::Vector2d direction_vector = rotation3 * (ICM - wheel_coord);
                double beita_i=atan2(direction_vector.y(), direction_vector.x());
                pathstate.V_body_direction.push_back(beita_i);

                double delta_i = (std::abs(beita_i) <= M_PI/2) ? beita_i : beita_i - std::copysign(M_PI, beita_i);
                pathstate.delta.push_back(delta_i);

                bool D_i = (int(beita_i / M_PI_2) == 0) ? false : true;
                pathstate.D.push_back(D_i);

                double v_i=(ICM - wheel_coord).norm()*womega;
                pathstate.V.push_back(v_i);
            }
            local_states.push_back(pathstate);

        }else{

            ICM_body_.push_back(std::nullopt);

            V_max_.push_back(HybridAStar::Constants::V_max_vision);

            time_interval_end_point_p = HybridAStar::Constants::V_max_vision * Tmax * original_vector/original_vector.norm();
            time_interval_end_points_.push_back(
                Eigen::Vector3d(
                    time_interval_end_point_p[0],
                    time_interval_end_point_p[1],
                    static_cast<double>(0)
                ));

            p_danger=keypoint_coords_body_[0];
            point_danger.push_back(p_danger);

            double beita_i=atan2(y,x);

            double delta_i = (std::abs(beita_i) <= M_PI_2) ? beita_i : (beita_i - std::copysign(M_PI, beita_i));

            bool D_i = (int(beita_i / M_PI_2) == 0) ? false : true;

            for (const auto& wheel_coord : wheel_coords_body_){
                pathstate.V_body_direction.push_back(beita_i);
                pathstate.delta.push_back(delta_i);
                pathstate.D.push_back(D_i);
                pathstate.V.push_back(HybridAStar::Constants::V_max_vision);
            }
            local_states.push_back(pathstate);
        }

    }

    yaw_res_max=omni_w_max*Tmax;
}

void SamplingResult::get_mode_shifting_SamplingTable()
{
    int sample_density=HybridAStar::Constants::sample_density;
    if (sample_density <= 1) {
        sample_density = 2;
    }

    const std::vector<double> steer_angles_deg = {10.0, 20.0, 30.0, 40.0, 0, -10.0, -20.0, -30.0, -40.0,};
    std::vector<double> steer_angles_rad;
    steer_angles_rad.reserve(steer_angles_deg.size());
    for (const double angle_deg : steer_angles_deg) {
        steer_angles_rad.push_back(angle_deg * M_PI / 180.0);
    }

    const double velocity = HybridAStar::Constants::V_max_vision;
    const double wheel_base = 3.5;
    const double duration = Tmax;
    const double dt = std::min(duration, 0.01);
    hexahedron_end_points_ = sampleBicycleEndPointsBySteeringAngles(
        steer_angles_rad, velocity, wheel_base, duration, dt);
    if (HybridAStar::Constants::reverse) {
        const std::vector<Eigen::Vector3d> reverse_end_points =
            sampleBicycleEndPointsBySteeringAngles(
                steer_angles_rad, -velocity, wheel_base, duration, dt);
        hexahedron_end_points_.insert(
            hexahedron_end_points_.end(), reverse_end_points.begin(), reverse_end_points.end());
    }

    hexahedron_end_points_.emplace_back(0.0, 0.0, 1.0);
    hexahedron_end_points_.emplace_back(0.0, 0.0, -1.0);

    const std::vector<double> diagonal_angles_deg = {45.0, -45.0, 135.0, -135.0};
    std::vector<double> diagonal_angles_rad;
    diagonal_angles_rad.reserve(diagonal_angles_deg.size());
    for (const double angle_deg : diagonal_angles_deg) {
        diagonal_angles_rad.push_back(angle_deg * M_PI / 180.0);
    }

    const std::vector<Eigen::Vector3d> diagonal_end_points =
        sampleDiagonalEndPointsByTravelAngles(diagonal_angles_rad, velocity, duration);
    hexahedron_end_points_.insert(
        hexahedron_end_points_.end(), diagonal_end_points.begin(), diagonal_end_points.end());

    num_points = static_cast<int>(hexahedron_end_points_.size());
    neighbor_g_.assign(num_points, std::vector<double>(num_points, 0.0));

    std::sort(hexahedron_end_points_.begin(), hexahedron_end_points_.end(),

        [](const Point3D& p1, const Point3D& p2) {
            bool p1_has_two_zeros = has_two_zero_coords(p1);
            bool p2_has_two_zeros = has_two_zero_coords(p2);

            if (p1_has_two_zeros && !p2_has_two_zeros) {
                return true;
            }

            return false;
        }
    );

    double R=0.0;
    Point2D time_interval_end_point_p;
    Point3D time_interval_end_point;
    Point2D p_danger;
    Eigen::Vector2d ICM;

    omni_w_max = 0.0f;
    bool find_omin=0;
    int photo_num=0;
    plt::figure();
    plt::xlim(-1, 1);
    plt::ylim(-1,1);
    for (const auto& p : hexahedron_end_points_) {
        double x = p[0];
        double y = p[1];
        double angle_rad = p[2];
        if(angle_rad>M_PI)angle_rad=angle_rad-M_PI*2.0f;
        Vector2D original_vector(x, y);
        PathState pathstate;
        if(angle_rad!=0){
            int8_t f = (angle_rad > 0) ? 1 : -1;

            Eigen::Matrix2d M_for_icm;
                M_for_icm << cos(angle_rad)-1, sin(angle_rad),
                    -1.0*sin(angle_rad), cos(angle_rad)-1;

            ICM = 1.0/(2*cos(angle_rad)-2) * (M_for_icm * original_vector) ;
            ICM_body_.push_back(ICM);

            R = ICM.norm();

            auto max_it = std::max_element(keypoint_coords_body_.begin(), keypoint_coords_body_.end(),
                [ICM](const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
                    return (p1 - ICM).squaredNorm() < (p2 - ICM).squaredNorm();
                }
            );
            p_danger=*max_it;
            point_danger.push_back(p_danger);
            double womega = HybridAStar::Constants::V_max_vision/(*max_it - ICM).norm();
            double theta = womega * Tmax;

            Eigen::Matrix2d M_for_newp;
            M_for_newp << cos(angle_rad)-1, -1.0*sin(angle_rad),
                sin(angle_rad), cos(angle_rad)-1;

            time_interval_end_point_p=-1.0* (M_for_newp * ICM) ;
            time_interval_end_points_.push_back(
                Eigen::Vector3d(
                    time_interval_end_point_p[0],
                    time_interval_end_point_p[1],
                    static_cast<double>(theta * f)
                ));

            double norm_value = ICM.norm();
            V_max_.push_back(womega*norm_value);

            if(find_omin==0 && time_interval_end_point_p[0]==0 && time_interval_end_point_p[1]==0){
                omni_w_max = womega;
                find_omin=1;
            }
            Eigen::Rotation2D<double> rotation3(-M_PI_2*f);
            for (const auto& wheel_coord : wheel_coords_body_){

                Eigen::Vector2d direction_vector = rotation3 * (ICM - wheel_coord);
                double beita_i=atan2(direction_vector.y(), direction_vector.x());
                pathstate.V_body_direction.push_back(beita_i);

                double delta_i = (std::abs(beita_i) <= M_PI/2) ? beita_i : beita_i - std::copysign(M_PI, beita_i);
                pathstate.delta.push_back(delta_i);

                bool D_i = (int(beita_i / M_PI_2) == 0) ? false : true;
                pathstate.D.push_back(D_i);

                double v_i=(ICM - wheel_coord).norm()*womega;
                pathstate.V.push_back(v_i);
            }
            local_states.push_back(pathstate);

        }else{

            ICM_body_.push_back(std::nullopt);

            V_max_.push_back(HybridAStar::Constants::V_max_vision);

            time_interval_end_point_p = HybridAStar::Constants::V_max_vision * Tmax * original_vector/original_vector.norm();
            time_interval_end_points_.push_back(
                Eigen::Vector3d(
                    time_interval_end_point_p[0],
                    time_interval_end_point_p[1],
                    static_cast<double>(0)
                ));

            p_danger=keypoint_coords_body_[0];
            point_danger.push_back(p_danger);

            double beita_i=atan2(y,x);

            double delta_i = (std::abs(beita_i) <= M_PI_2) ? beita_i : (beita_i - std::copysign(M_PI, beita_i));

            bool D_i = (int(beita_i / M_PI_2) == 0) ? false : true;

            for (const auto& wheel_coord : wheel_coords_body_){
                pathstate.V_body_direction.push_back(beita_i);
                pathstate.delta.push_back(delta_i);
                pathstate.D.push_back(D_i);
                pathstate.V.push_back(HybridAStar::Constants::V_max_vision);
            }
            local_states.push_back(pathstate);
        }

    }

    yaw_res_max=omni_w_max*Tmax;
}

void SamplingResult::get_neighbor_ordi_g()
{

    size_t num_ordinary = ordinary_hybrid_end_points_.size();
    neighbor_ordi_g_.resize(num_ordinary);
    for (auto& row : neighbor_ordi_g_) {
        row.resize(num_ordinary);
    }

    double V_max = HybridAStar::Constants::V_max_vision;
    double amax = HybridAStar::Constants::amax;
    double adec = HybridAStar::Constants::adec;
    double w_max = HybridAStar::Constants::w_max_wheel;
    double xy_resolution = 2*1.0 / HybridAStar::Constants::positionResolution;
    double yaw_resolution = 2 * HybridAStar::Constants::deltaHeadingRad;

    double motor_for_v_up = V_max/amax;
    double motor_for_v_down = V_max/adec;
    double motor_for_w = 0.5*M_PI/w_max;

    double tv=xy_resolution/V_max;
    double tyaw=yaw_resolution/w_max;

    for(size_t i=0;i<num_ordinary;i++)
    {
        for(size_t j=0;j<num_ordinary;j++)
        {

            if(i==j){
                neighbor_ordi_g_[i][j]=0;
            }else if(i<2 && j<2){
                neighbor_ordi_g_[i][j]=motor_for_v_up + motor_for_v_down;
            }else if(i>1 && j>1){
                neighbor_ordi_g_[i][j]=motor_for_v_up + motor_for_v_down;
            }else{
                neighbor_ordi_g_[i][j]=motor_for_v_up + motor_for_v_down + motor_for_w;
            }

            if(j<2){
                neighbor_ordi_g_[i][j]+=tv;
            }else{
                neighbor_ordi_g_[i][j]+=tyaw;
            }
        }
    }
}

void SamplingResult::get_neighbor_g()
{
    double acc[2]={HybridAStar::Constants::amax,HybridAStar::Constants::adec};

    for(int i=0;i<num_points;i++){
        for(int j=0;j<num_points;j++){
            if(i==13&&j==11){
                printf("Debug here!\r\n");
            }

            double T_change_v=0.0;
            double T_action=Tmax;
            double T_change_wheel=0.0;

            bool isdoublefish=0;
            bool isbigfish=0;
            bool islittlefish=1;

            double Tmax_cost=0.0;

            for(int tip_wheel=0;tip_wheel<num_wheels;tip_wheel++){
                if((std::abs(local_states[i].V_body_direction[tip_wheel])-M_PI_2)*
                    (std::abs(local_states[j].V_body_direction[tip_wheel])-M_PI_2)
                    <0
                    ){
                        isdoublefish=1;
                    }
                else if(std::abs(local_states[i].V_body_direction[tip_wheel]-
                local_states[j].V_body_direction[tip_wheel])>(HybridAStar::Constants::w_max_wheel*2*Tmax))
                isbigfish=1;
            }

            for(int tip_wheel=0;tip_wheel<num_wheels;tip_wheel++){

                double vi=std::abs(local_states[i].V[tip_wheel]);
                double vj=std::abs(local_states[j].V[tip_wheel]);
                double d_vij=vi-vj;
                double diff=std::abs(local_states[i].V_body_direction[tip_wheel] - std::abs(local_states[j].V_body_direction[tip_wheel]));
                double v_change_angle= std::min(diff, 2 * M_PI - diff);

                double t_d_vij=0.0;
                if(d_vij>0){t_d_vij=d_vij/acc[1];}else{t_d_vij=-d_vij/acc[0];}

                double d_dw_ij=std::abs(local_states[i].delta[tip_wheel]
                -local_states[j].delta[tip_wheel]);

                if(isdoublefish==1){
                    T_change_v=vi/acc[1]+vj/acc[0];
                    T_change_wheel=d_dw_ij/HybridAStar::Constants::w_max_wheel-vj/acc[0];
                    T_change_wheel=(T_change_wheel<0)?0:(T_change_wheel);
                    double T_this=T_change_v+T_action+T_change_wheel
                                   +HybridAStar::Constants::angle_wheel_change_ratio*d_dw_ij
                                   +HybridAStar::Constants::angle_v_change_ratio*v_change_angle;

                    if(T_this>Tmax_cost){
                        Tmax_cost = T_this;
                    }
                }else if(isbigfish==1){
                    T_change_v=vi/acc[1]+vj/acc[0];
                    T_change_wheel=d_dw_ij/HybridAStar::Constants::w_max_wheel-T_change_v;
                    T_change_wheel=(T_change_wheel<0)?0:(T_change_wheel)
                                   +HybridAStar::Constants::angle_wheel_change_ratio*d_dw_ij
                                   +HybridAStar::Constants::angle_v_change_ratio*v_change_angle;
                    double T_this=T_change_v+T_action+T_change_wheel;
                    if(T_this>Tmax_cost){
                        Tmax_cost = T_this;
                    }
                }else{
                    T_change_v=t_d_vij;
                    T_change_wheel=0;
                    double T_this=T_change_v+T_action+T_change_wheel
                                   +HybridAStar::Constants::angle_wheel_change_ratio*d_dw_ij
                                   +HybridAStar::Constants::angle_v_change_ratio*v_change_angle;
                    if(T_this>Tmax_cost){
                        Tmax_cost = T_this;
                    }
                }
            }
            if(HybridAStar::Constants::coutDEBUG_M){
                printf("neighbor_g_[%d][%d] ： T = %.4f \r\n",i,j,Tmax_cost);
            }
            neighbor_g_[i][j] = Tmax_cost ;
        }
    }
}

void SamplingResult::output_samples_to_json() const {
    if (hexahedron_end_points_.size() != ICM_body_.size()) {
        std::cerr << "[SamplingResult] hexahedron_end_points_ and ICM_body_ size mismatch, skip exporting." << std::endl;
        return;
    }

    namespace fs = std::filesystem;
    const fs::path output_dir = fs::path(HybridAStar::Constants::Experiment_Output_Root) /
                                HybridAStar::Constants::Experiment_Method /
                                HybridAStar::Constants::Experiment_Scene /
                                HybridAStar::Constants::Experiment_Map /
                                "sample";

    std::error_code ec;
    fs::create_directories(output_dir, ec);
    if (ec) {
        std::cerr << "[SamplingResult] Failed to create directory: " << output_dir << " (" << ec.message() << ")" << std::endl;
        return;
    }

    const std::string filename = "sample_" + HybridAStar::Constants::Experiment_Method + "_" +
                                 HybridAStar::Constants::Experiment_Scene + "_" +
                                 HybridAStar::Constants::Experiment_Map +
                                 "_density" + std::to_string(HybridAStar::Constants::sample_density) + ".json";
    const fs::path output_file = output_dir / filename;

    std::ofstream ofs(output_file);
    if (!ofs.is_open()) {
        std::cerr << "[SamplingResult] Failed to open file for writing: " << output_file << std::endl;
        return;
    }

    ofs << std::fixed << std::setprecision(6);
    ofs << "{\n";
    ofs << "  \"sample_density\": " << HybridAStar::Constants::sample_density << ",\n";
    ofs << "  \"num_points\": " << hexahedron_end_points_.size() << ",\n";
    ofs << "  \"points\": [\n";

    for (size_t i = 0; i < hexahedron_end_points_.size(); ++i) {
        const auto& point = hexahedron_end_points_[i];
        ofs << "    {\n";
        ofs << "      \"index\": " << i << ",\n";
        ofs << "      \"hexahedron_end_point\": {"
            << "\"x\": " << point.x() << ", "
            << "\"y\": " << point.y() << ", "
            << "\"z\": " << point.z() << "},\n";
        ofs << "      \"icm_body\": ";
        if (ICM_body_[i].has_value()) {
            ofs << "{"
                << "\"x\": " << ICM_body_[i]->x() << ", "
                << "\"y\": " << ICM_body_[i]->y()
                << "}\n";
        } else {
            ofs << "null\n";
        }
        ofs << "    }";
        if (i + 1 != hexahedron_end_points_.size()) {
            ofs << ",";
        }
        ofs << "\n";
    }

    ofs << "  ]\n";
    ofs << "}\n";

    if (HybridAStar::Constants::coutDEBUG_M) {
        std::cout << "[SamplingResult] Exported sampling table to " << output_file << std::endl;
    }
}

double crossProduct(const Point2D& p0, const Point2D& p1, const Point2D& p2) {
    return (p1.x() - p0.x()) * (p2.y() - p0.y()) - (p2.x() - p0.x()) * (p1.y() - p0.y());
}

double distanceSquared(const Point2D& p1, const Point2D& p2) {
    double dx = p1.x() - p2.x();
    double dy = p1.y() - p2.y();
    return dx * dx + dy * dy;
}

void SamplingResult::getkeyPoint() {

    const std::vector<Eigen::Vector2d> &car_shape=HybridAStar::Constants::my_car_shape_for_show();

    if (car_shape.size() < 3) {
        keypoint_coords_body_= car_shape;
        return;
    }

    Point2D pivot = car_shape[0];
    int pivot_index = 0;
    for (size_t i = 1; i < car_shape.size(); ++i) {
        if (car_shape[i].y() < pivot.y() ||
            (std::abs(car_shape[i].y() - pivot.y()) < 1e-9 && car_shape[i].x() < pivot.x())) {
            pivot = car_shape[i];
            pivot_index = i;
        }
    }

    std::vector<Point2D> sorted_points = car_shape;
    std::swap(sorted_points[0], sorted_points[pivot_index]);

    auto comparator = [&pivot](const Point2D& a, const Point2D& b) {
        double cross = crossProduct(pivot, a, b);
        if (std::abs(cross) < 1e-9) {

            return distanceSquared(pivot, a) < distanceSquared(pivot, b);
        }

        return cross > 0;
    };
    std::sort(sorted_points.begin() + 1, sorted_points.end(), comparator);

    std::vector<Point2D> convex_hull;
    convex_hull.push_back(sorted_points[0]);
    convex_hull.push_back(sorted_points[1]);

    for (size_t i = 2; i < sorted_points.size(); ++i) {

        while (convex_hull.size() > 1 &&
               crossProduct(convex_hull[convex_hull.size() - 2], convex_hull.back(), sorted_points[i]) <= 0) {
            convex_hull.pop_back();
        }
        convex_hull.push_back(sorted_points[i]);
    }

    keypoint_coords_body_=convex_hull;

}

    SamplingResult SamplingResult::createWithDefaults() {
        spmt_visual::PARAMS params;
        if(HybridAStar::Constants::Algorithm_Framework_underothers == "mode_shifting"){
            const int bicycle_sample_count = HybridAStar::Constants::reverse ? 18 : 9;
            return SamplingResult(
                bicycle_sample_count+2+4,
                params.n_axil*2,
                HybridAStar::Constants::wheel_coords(),
                HybridAStar::Constants::my_car_shape_for_show()
            );
        }
        return SamplingResult(
            6 * std::pow(HybridAStar::Constants::sample_density, 2) + 2,
            params.n_axil*2,
            HybridAStar::Constants::wheel_coords(),
            HybridAStar::Constants::my_car_shape_for_show()
        );
    }

    void SamplingResult::show_car_outline() {

        std::vector<double> x, y;
        for (const auto& point : HybridAStar::Constants::my_car_shape_for_show()) {
            x.push_back(point.x());
            y.push_back(point.y());
        }
        plt::plot(x,y,"b-");

        std::vector<double> xx, yy;
        for (const auto& point : HybridAStar::Constants::wheel_coords()) {
            xx.push_back(point.x());
            yy.push_back(point.y());
        }
        plt::scatter(xx,yy);
    }
