#ifndef SAMPLING_M_H
#define SAMPLING_M_H

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "constants.h"
#include "spmt_model.h"
#include"matplotlibcpp.h"

class  SamplingResult {
public:

    SamplingResult(size_t num_points,
                   size_t num_wheels,
                   const std::vector<Eigen::Vector2d>& wheel_coords,
                   const std::vector<Eigen::Vector2d>& keypoint_coords );

    ~SamplingResult() = default;

    SamplingResult(const SamplingResult&) = delete;
    SamplingResult& operator=(const SamplingResult&) = delete;
    SamplingResult(SamplingResult&&) = delete;
    SamplingResult& operator=(SamplingResult&&) = delete;

    static SamplingResult createWithDefaults();

    static inline Eigen::Vector3d integrateBicycleKinematics(
        double steer_angle,
        double velocity,
        double wheel_base,
        double duration,
        double dt,
        const Eigen::Vector3d& start = Eigen::Vector3d::Zero()) {

        if (wheel_base <= 0.0) {
            throw std::invalid_argument("wheel_base must be positive");
        }
        if (duration < 0.0) {
            throw std::invalid_argument("duration must be non-negative");
        }
        if (dt <= 0.0) {
            throw std::invalid_argument("dt must be positive");
        }

        Eigen::Vector3d state = start;
        double elapsed = 0.0;

        while (elapsed < duration) {
            const double h = std::min(dt, duration - elapsed);
            const double yaw = state.z();
            const double yaw_rate = velocity / wheel_base * std::tan(steer_angle);

            state.x() += velocity * std::cos(yaw) * h;
            state.y() += velocity * std::sin(yaw) * h;
            state.z() = normalizeAngle(state.z() + yaw_rate * h);

            elapsed += h;
        }

        return state;
    }

    static inline Eigen::Vector3d integrateDiagonalKinematics(
        double travel_angle,
        double velocity,
        double duration,
        const Eigen::Vector3d& start = Eigen::Vector3d::Zero()) {

        if (duration < 0.0) {
            throw std::invalid_argument("duration must be non-negative");
        }

        Eigen::Vector3d state = start;
        state.x() += velocity * std::cos(travel_angle) * duration;
        state.y() += velocity * std::sin(travel_angle) * duration;
        state.z() = normalizeAngle(state.z());
        return state;
    }

    static inline std::vector<Eigen::Vector3d> sampleDiagonalEndPointsByTravelAngles(
        const std::vector<double>& travel_angles,
        double velocity,
        double duration,
        const Eigen::Vector3d& start = Eigen::Vector3d::Zero()) {

        std::vector<Eigen::Vector3d> endpoints;
        endpoints.reserve(travel_angles.size());

        for (const double travel_angle : travel_angles) {
            endpoints.emplace_back(integrateDiagonalKinematics(
                travel_angle, velocity, duration, start));
        }

        return endpoints;
    }

    static inline std::vector<Eigen::Vector3d> sampleBicycleEndPointsBySteeringAngles(
        const std::vector<double>& steer_angles,
        double velocity,
        double wheel_base,
        double duration,
        double dt,
        const Eigen::Vector3d& start = Eigen::Vector3d::Zero()) {

        std::vector<Eigen::Vector3d> endpoints;
        endpoints.reserve(steer_angles.size());

        for (const double steer_angle : steer_angles) {
            endpoints.emplace_back(integrateBicycleKinematics(
                steer_angle, velocity, wheel_base, duration, dt, start));
        }

        return endpoints;
    }

    const std::vector<Eigen::Vector3d>& getHexahedronEndPoints() const { return hexahedron_end_points_; }
    const std::vector<Eigen::Vector3d>& getTimeIntervalEndPoints() const { return time_interval_end_points_; }
    const std::vector<Eigen::Vector3d>& get_ordi_EndPoints() const { return ordinary_hybrid_end_points_; }

    void debug_discrete_compare() {
        for (size_t i = 0; i < hexahedron_end_points_.size(); ++i) {
            std::cout << "  hex_points [" << i << "]: (x: " << hexahedron_end_points_[i].x()
                    << ", y: " << hexahedron_end_points_[i].y()
                    << ", z: " << hexahedron_end_points_[i].z() << ")" << std::endl;

            std::cout << "  time_points [" << i << "]: (x: " << time_interval_end_points_[i].x()
                    << ", y: " << time_interval_end_points_[i].y()
                    << ", z: " << time_interval_end_points_[i].z() << ")" << std::endl;

            std::cout << "  V_max_ [" << i << "]: ( " << V_max_[i] <<")" << std::endl;
        }
    }

    float getMaxVelocity(int i) const { return V_max_[i]; }
    float getnumpoints() const { if(HybridAStar::Constants::fix_axil)return ordinary_hybrid_end_points_.size();else return num_points; }
    double getomni_w_max()const { return omni_w_max; }
    double get_Tmax()const { return Tmax; }
    std::vector<std::optional<Eigen::Vector2d>> getICM() const { return ICM_body_; }

    std::vector<double>& getWheelVelocityLast(int i) { return local_states[i].V_body_direction; }

    const std::vector<bool>& getWheelDirections(int i) const { return local_states[i].D; }
    const std::vector<float>& getWheelPhysicalAngles(int i) const { return local_states[i].delta; }

    const std::vector<Eigen::Vector2d>& getWheelCoords() const { return wheel_coords_body_; }
    const std::vector<Eigen::Vector2d>& getKeypointCoords() const { return keypoint_coords_body_; }

   std::vector<std::vector<double>> getneighbor_g_() { return neighbor_g_; }
   std::vector<std::vector<double>> getneighbor_ordi_g_() { return neighbor_ordi_g_; }

    void getkeyPoint();

    void getAWSConstructSamplingTable() ;

    void get_hyAstar_SamplingTable() ;

    void get_mode_shifting_SamplingTable() ;

    void get_neighbor_g();

    void get_neighbor_ordi_g();

    void output_samples_to_json() const;

    void show_car_outline();

    struct PathState {

        std::vector<double> V_body_direction;

        std::vector<bool> D;

        std::vector<float> delta;

        std::vector<double> V;

        PathState() = default;

        PathState(
                  const std::vector<bool>& d,
                  const std::vector<float>& delta_vals,
                  const std::optional<Eigen::Vector2d>& icm)
            :
              D(d),
              delta(delta_vals){}
    };

private:
    static inline double normalizeAngle(double angle) {
        constexpr double two_pi = 2.0 * M_PI;
        angle = std::fmod(angle + M_PI, two_pi);
        if (angle < 0.0) {
            angle += two_pi;
        }
        return angle - M_PI;
    }

    float Tmax ;

    std::vector<Eigen::Vector3d> hexahedron_end_points_;

    std::vector<Eigen::Vector3d> time_interval_end_points_;

    std::vector<Eigen::Vector3d> ordinary_hybrid_end_points_;

    std::vector<float>  V_max_;

    std::vector<Eigen::Vector2d>  point_danger;

    std::vector<std::optional<Eigen::Vector2d>> ICM_body_;

    const std::vector<Eigen::Vector2d> wheel_coords_body_;

    std::vector<Eigen::Vector2d> keypoint_coords_body_;

    size_t num_wheels;
    int num_points;

    std::vector<PathState> local_states;

    std::vector<std::vector<double>> neighbor_g_;
    std::vector<std::vector<double>> neighbor_ordi_g_;

    float omni_w_max;

    float yaw_res_max;
};

#endif
