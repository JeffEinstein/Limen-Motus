#ifndef SPMT_MODEL_H
#define SPMT_MODEL_H
#include"matplotlibcpp.h"
#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <nlohmann/json.hpp>
#include <limen_motus/car_control.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <fstream>
namespace plt =matplotlibcpp;
#include <unistd.h>
namespace spmt_visual
{
    struct Point {
    float x;
    float y;
    };

    struct Tire{
        float wid_s = 0.355;
        float wid= 0.62+wid_s;
        float len= 0.9435;
    };
    struct Powerhead{
        float wid_car=2.43;
        float len_forward=3.40;
    };
    struct PARAMS{
        const int n_axil=6;
        const float l_axleDistance=1.4;
        const float l_axleDistance_half=l_axleDistance/2.0;
        const float b_trackWidth=1.45;
        const float width=2.43;
        Point positions[2][7];
        Tire tire_params;
        Powerhead head_params;
    };
    struct STATE{
        double x,y,yaw;
        float w_c,v_c,Mode_c,shaftcut_c,level_c,duration;
        float max_angle;
        float angles[2][6];
        float angles_vel[2][6];
        Point positions[2][7];
        Point cir;
        int forward;
        int yaw_direction;
        Point cir_car;

        Point positions_slide[2][7];
    };

    class spmt
    {
    private:

        ros::NodeHandle ros_nh_;
        ros::Subscriber car_control_sub_;

        ros::Subscriber Map_sub_;
        ros::Subscriber Start_sub_;
        ros::Subscriber Goal_sub_;

        std::string diver_topic_="car_contrl";
        std::string Map_topic_="/map";
        void OnControlMsg(const limen_motus::car_control::ConstPtr& msg);
        void GetMap(const nav_msgs::OccupancyGrid::Ptr& map);
        void startCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

        int map_height;
        int map_width;
        bool** binMap;
        bool Have_Map;

        PARAMS params;

        STATE last_state;
        STATE cur_state;
        STATE start_state;
        bool start_geted;

        int const_angle_for_paper=-91;
        bool paper_first_experiment_isgoing=false;
        bool data_getted=false ;
        int radius_simulate_slide=1e8;
        int n_simulate_slide=0;
        Point o_simulate_slide={-n_simulate_slide*params.l_axleDistance_half,0.5*params.width};

        bool enable_visualization=true;

        double calculateClockwiseAngle(const Point& p);
        long double reshap_otopi(double angle);
    public:
        spmt();
        virtual ~spmt();
        void init_firstplace();
        void node_begin();
        void angles_get_ackman();
        void angles_get_slide();
        void get_positions();
        void show_car();
        void visualizeBinaryMap();

    };
}
#endif
