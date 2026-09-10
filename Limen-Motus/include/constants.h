#ifndef CONSTANTS
#define CONSTANTS

#define othersmethod

#include <cmath>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "spmt_model.h"

namespace HybridAStar {

namespace Constants {

static const bool fix_axil = false;
static const std::string Convex_Method = "triangle";

static const std::string Algorithm_Framework = "AWS_from_others";
static const std::string Algorithm_Framework_underothers = "complete_aws";

static const bool output_samples = true;
static const std::string Experiment_Method = "method3_complete_aws";
static const std::string Experiment_Scene = "ant";
static const std::string Experiment_Map = "ant15";
static const std::string Experiment_Output_Root = "/workspaces/ros-noetic-ws/src/Test_Serial/SPMT_RESEARCH/src/Limen-Motus/experiment_results";
static const bool Experiment_Run_One_Pair = true;
static const bool Experiment_Use_Onsite_450_Pairs = false;
static const std::string Experiment_Onsite_450_Pairs_File = "";

static const bool coutDEBUG_M = false;

static const bool coutDEBUG = false;

static const bool manual = true;

static const bool visualization = false || manual;

static const bool visualization2D = false || manual;

static const bool reverse = true;

static const bool dubinsShot = true;

static const bool dubins = false;

static const bool dubinsLookup = false && dubins;

static const bool twoD = true;

inline const double& bloating() {
    static const double bloating = 0;
    return bloating;
}

inline const double& axles() {
  spmt_visual::PARAMS params;
    static const double value = params.n_axil;
    return value;
}

inline const double& width() {
  spmt_visual::PARAMS params;

    static const double value = params.width + 2 * bloating();
    return value;
}

inline const double& length() {
  spmt_visual::PARAMS params;
  spmt_visual::Powerhead head;
  static const double value = axles() * params.l_axleDistance +  head.len_forward + 2 * bloating();
  return value;
}

inline const std::vector<Eigen::Vector2d>& my_spmt_shape() {
    spmt_visual::PARAMS params;
    spmt_visual::Powerhead powerhead;
    static const std::vector<Eigen::Vector2d> shape = {

    {5.9, 1.215},
    {-5.9, 1.215},
    {-5.9, -1.215},
    {5.9, -1.215}
};
    return shape;
}

inline const std::vector<Eigen::Vector2d>& my_cargo_shape() {
    spmt_visual::PARAMS params;
    spmt_visual::Powerhead powerhead;
    static const std::vector<Eigen::Vector2d> shape = {

    {5.9,-1.215},
    {5.9, 1.215},
    {0.4, 1.215},
    {0.4, 3.215},
    {-3.8,3.215},
    {-3.8,1.215},
    {-5.9,1.215},
    {-5.9,-1.215},
    {-3.8,-1.215},
    {-3.8,-3.215},
    { 0.4,-3.215},
    { 0.4,-1.215}
};
    return shape;
}

inline const std::vector<Eigen::Vector2d>& piano_shape() {
    spmt_visual::PARAMS params;
    spmt_visual::Powerhead powerhead;
    static const std::vector<Eigen::Vector2d> shape = {

      { 6*2.0,-1.5*2.0},
      { 6*2.0,-2.5*2.0},
      { 8*2.0,-2.5*2.0},
      { 8*2.0,2.5 *2.0},
      { 6*2.0,2.5 *2.0},
      { 6*2.0,1.5 *2.0},
      {-6*2.0,1.5 *2.0},
      {-6*2.0,4.5 *2.0},
      {-8*2.0,4.5 *2.0},
      {-8*2.0,-4.5*2.0},
      {-6*2.0,-4.5*2.0},
      {-6*2.0,-1.5*2.0}
};
    return shape;
}
inline const std::vector<Eigen::Vector2d>& plane_shape() {
    static const std::vector<Eigen::Vector2d> shape = {

        {-2.5*2.0, 0},

        {-2.278988191*2.0, -0.530987179*2.0},
        {-0.859316863*2.0, -0.530987179*2.0},

        {0.369910749*2.0, -2.573928845*2.0},
        {1.088402945*2.0, -2.573928845*2.0},

        {0.612293659*2.0, -0.530987179*2.0},

        {1.936743129*2.0, -0.409795724*2.0},
        {2.406118541*2.0, -1.189884439*2.0},

        {2.776426779*2.0, -1.189884439*2.0},
        {2.5*2.0, 0},

        {2.776426779*2.0, 1.189884439*2.0},
        {2.406118541*2.0, 1.189884439*2.0},

        {1.936743129*2.0, 0.409795724*2.0},
        {0.612293659*2.0, 0.530987179*2.0},

        {1.088402945*2.0, 2.573928845*2.0},
        {0.369910749*2.0, 2.573928845*2.0},

        {-0.859316863*2.0, 0.530987179*2.0},
        {-2.278988191*2.0, 0.530987179*2.0}
    };
    return shape;
}

inline const std::vector<Eigen::Vector2d>& piano_shape_ellipse() {
    static const std::vector<Eigen::Vector2d> shape = {
      {-14.0, 0.0},
      {-11.642977705123142, 7.954951067301105},

      {0.0, 0.0},
      {11.9999999303873, 3.0000001544573225},

      {14.0, 0.0},
      {15.99999977910198, 5.000000157424837},
};
    return shape;
}

inline const std::vector<Eigen::Vector2d>& cargo_shape_ellipse() {
    static const std::vector<Eigen::Vector2d> shape = {
      {0.0, 0.0},
      {5.9, 1.215},

      {-1.7, 0.0},
      {0.4, 3.215},
};
    return shape;
}

inline const std::vector<Eigen::Vector2d>& cargo_shape_ellispse() {
    return cargo_shape_ellipse();
}

inline const std::vector<Eigen::Vector2d>& my_car_shape_for_collision() {
  return piano_shape();
}

inline const std::vector<Eigen::Vector2d>& my_car_shape_for_show() {
  return piano_shape();
}

const Eigen::Vector2d xyofset_SPMT={2.5,-1.215};

inline const std::vector<Eigen::Vector2d>& wheel_coords() {
    spmt_visual::PARAMS params;
    static const std::vector<Eigen::Vector2d> value = {

    { -1 * params.l_axleDistance_half - 0 * params.l_axleDistance +xyofset_SPMT[0], xyofset_SPMT[1]+(params.width-params.b_trackWidth)/2.0},
    { -1 * params.l_axleDistance_half - 0 * params.l_axleDistance +xyofset_SPMT[0], xyofset_SPMT[1]+(params.width-params.b_trackWidth)/2.0+params.b_trackWidth},
    { -1 * params.l_axleDistance_half - 1 * params.l_axleDistance +xyofset_SPMT[0], xyofset_SPMT[1]+(params.width-params.b_trackWidth)/2.0},
    { -1 * params.l_axleDistance_half - 1 * params.l_axleDistance +xyofset_SPMT[0], xyofset_SPMT[1]+(params.width-params.b_trackWidth)/2.0+params.b_trackWidth},
    { -1 * params.l_axleDistance_half - 2 * params.l_axleDistance +xyofset_SPMT[0], xyofset_SPMT[1]+(params.width-params.b_trackWidth)/2.0},
    { -1 * params.l_axleDistance_half - 2 * params.l_axleDistance +xyofset_SPMT[0], xyofset_SPMT[1]+(params.width-params.b_trackWidth)/2.0+params.b_trackWidth},
    { -1 * params.l_axleDistance_half - 3 * params.l_axleDistance +xyofset_SPMT[0], xyofset_SPMT[1]+(params.width-params.b_trackWidth)/2.0},
    { -1 * params.l_axleDistance_half - 3 * params.l_axleDistance +xyofset_SPMT[0], xyofset_SPMT[1]+(params.width-params.b_trackWidth)/2.0+params.b_trackWidth},
    { -1 * params.l_axleDistance_half - 4 * params.l_axleDistance +xyofset_SPMT[0], xyofset_SPMT[1]+(params.width-params.b_trackWidth)/2.0},
    { -1 * params.l_axleDistance_half - 4 * params.l_axleDistance +xyofset_SPMT[0], xyofset_SPMT[1]+(params.width-params.b_trackWidth)/2.0+params.b_trackWidth},
    { -1 * params.l_axleDistance_half - 5 * params.l_axleDistance +xyofset_SPMT[0], xyofset_SPMT[1]+(params.width-params.b_trackWidth)/2.0},
    { -1 * params.l_axleDistance_half - 5 * params.l_axleDistance +xyofset_SPMT[0], xyofset_SPMT[1]+(params.width-params.b_trackWidth)/2.0+params.b_trackWidth}

    };
    return value;
}

static const int iterations = 150000000/26;

static const float V_max_vision =0.5;

static const float w_max_wheel =M_PI/10;

static const float amax = 1;
static const float adec = 2;

static const float angle_wheel_change_ratio = 0;
static const float angle_v_change_ratio = 0;

static const  float sample_ratio = 3.0;

static const float r = 0.725;

static const float real_map = 1;

inline const std::vector<Eigen::Vector3d>& ant_sg() {
    static const std::vector<Eigen::Vector3d> value = {
      {21.578899383544922, 82.40519714355469, 4.81238899230957},
      {90.69290161132812, 72.20040130615234, 1.5707963705062866}
    };
    return value;
  }

inline const std::vector<Eigen::Vector3d>& uturn_green_sg() {
    static const std::vector<Eigen::Vector3d> value = {
      {3.51471, 58.0831, 1.5*M_PI+0.0001},
      {15.5043, 32.4834, 0},
      {33.9791, 58.216, 0.5*M_PI}
    };
    return value ;
  }

inline const std::vector<Eigen::Vector3d>& uturn_cross_sg() {
    static const std::vector<Eigen::Vector3d> value = {
      { 3, 34, 1.5*M_PI+0.0001},
      {16, 34, 0.5*M_PI}
    };
    return value ;
  }

inline const std::vector<Eigen::Vector3d>& onsite_sg() {
    static const std::vector<Eigen::Vector3d> value = {
      {186.883, 26.1403, 0.337258},
      {184.607, 16.4935, 5.58873}
    };
    return value ;
  }

inline const std::vector<Eigen::Vector3d>& parking_plot() {

    static const std::vector<Eigen::Vector3d> value = {

    {2.5 + 0.0*4.0 ,  7.5 ,(1.5*M_PI+0.0001)},
    {2.5 + 1.0*4.0 ,  7.5 ,(1.5*M_PI+0.0001)},
    {2.5 + 2.0*4.0 ,  7.5 ,(1.5*M_PI+0.0001)},
    {2.5 + 3.0*4.0 ,  7.5 ,(1.5*M_PI+0.0001)},
    {2.5 + 4.0*4.0 ,  7.5 ,(1.5*M_PI+0.0001)},
    {2.5 + 5.0*4.0 ,  7.5 ,(1.5*M_PI+0.0001)},
    {2.5 + 6.0*4.0 ,  7.5 ,(1.5*M_PI+0.0001)},
    {2.5 + 7.0*4.0 ,  7.5 ,(1.5*M_PI+0.0001)},
    {2.5 + 8.0*4.0 ,  7.5 ,(1.5*M_PI+0.0001)},
    {2.5 + 9.0*4.0 ,  7.5 ,(1.5*M_PI+0.0001)},
    {2.5 + 10.0*4.0 , 7.5 ,(1.5*M_PI+0.0001)},
    {2.5 + 11.0*4.0 , 7.5 ,(1.5*M_PI+0.0001)},
    {2.5 + 12.0*4.0 , 7.5 ,(1.5*M_PI+0.0001)},
    {2.5 + 13.0*4.0 , 7.5 ,(1.5*M_PI+0.0001)},
    {2.5 + 14.0*4.0 , 7.5 ,(1.5*M_PI+0.0001)},
    {2.5 + 15.0*4.0 , 7.5 ,(1.5*M_PI+0.0001)},
    {2.5 + 16.0*4.0 , 7.5 ,(1.5*M_PI+0.0001)},
    {2.5 + 17.0*4.0 , 7.5 ,(1.5*M_PI+0.0001)},
    {2.5 + 18.0*4.0 , 7.5 ,(1.5*M_PI+0.0001)},
    {2.5 + 19.0*4.0 , 7.5 ,(1.5*M_PI+0.0001)},

    {9.5 + 0.0*4.0 ,  34.5 ,(1.5*M_PI+0.0001)},
    {9.5 + 1.0*4.0 ,  34.5 ,(1.5*M_PI+0.0001)},
    {9.5 + 2.0*4.0 ,  34.5 ,(1.5*M_PI+0.0001)},
    {9.5 + 3.0*4.0 ,  34.5 ,(1.5*M_PI+0.0001)},
    {9.5 + 4.0*4.0 ,  34.5 ,(1.5*M_PI+0.0001)},
    {9.5 + 5.0*4.0 ,  34.5 ,(1.5*M_PI+0.0001)},
    {9.5 + 6.0*4.0 ,  34.5 ,(1.5*M_PI+0.0001)},
    {9.5 + 7.0*4.0 ,  34.5 ,(1.5*M_PI+0.0001)},
    {9.5 + 8.0*4.0 ,  34.5 ,(1.5*M_PI+0.0001)},
    {9.5 + 9.0*4.0 ,  34.5 ,(1.5*M_PI+0.0001)},
    {9.5 + 10.0*4.0 , 34.5 ,(1.5*M_PI+0.0001)},
    {9.5 + 11.0*4.0 , 34.5 ,(1.5*M_PI+0.0001)},
    {9.5 + 12.0*4.0 , 34.5 ,(1.5*M_PI+0.0001)},
    {9.5 + 13.0*4.0 , 34.5 ,(1.5*M_PI+0.0001)},
    {9.5 + 14.0*4.0 , 34.5 ,(1.5*M_PI+0.0001)},
    {9.5 + 15.0*4.0 , 34.5 ,(1.5*M_PI+0.0001)},
    {9.5 + 16.0*4.0 , 34.5 ,(1.5*M_PI+0.0001)},
    {9.5 + 17.0*4.0 , 34.5 ,(1.5*M_PI+0.0001)},

    {9.5 + 0.0*4.0 ,  48.5, 1.5*M_PI+0.0001},
    {9.5 + 1.0*4.0 ,  48.5, 1.5*M_PI+0.0001},
    {9.5 + 2.0*4.0 ,  48.5, 1.5*M_PI+0.0001},
    {9.5 + 3.0*4.0 ,  48.5, 1.5*M_PI+0.0001},
    {9.5 + 4.0*4.0 ,  48.5, 1.5*M_PI+0.0001},
    {9.5 + 5.0*4.0 ,  48.5, 1.5*M_PI+0.0001},
    {9.5 + 6.0*4.0 ,  48.5, 1.5*M_PI+0.0001},
    {9.5 + 7.0*4.0 ,  48.5, 1.5*M_PI+0.0001},
    {9.5 + 8.0*4.0 ,  48.5, 1.5*M_PI+0.0001},
    {9.5 + 9.0*4.0 ,  48.5, 1.5*M_PI+0.0001},
    {9.5 + 10.0*4.0 , 48.5, 1.5*M_PI+0.0001},
    {9.5 + 11.0*4.0 , 48.5, 1.5*M_PI+0.0001},
    {9.5 + 12.0*4.0 , 48.5, 1.5*M_PI+0.0001},
    {9.5 + 13.0*4.0 , 48.5, 1.5*M_PI+0.0001},
    {9.5 + 14.0*4.0 , 48.5, 1.5*M_PI+0.0001},
    {9.5 + 15.0*4.0 , 48.5, 1.5*M_PI+0.0001},
    {9.5 + 16.0*4.0 , 48.5, 1.5*M_PI+0.0001},
    {9.5 + 17.0*4.0 , 48.5, 1.5*M_PI+0.0001},

    {100.5 ,  2.5+ 0.0* 4.0  , M_PI},
    {100.5 ,  2.5+ 1.0* 4.0  , M_PI},
    {100.5 ,  2.5+ 2.0* 4.0  , M_PI},
    {100.5 ,  2.5+ 3.0* 4.0  , M_PI},
    {100.5 ,  2.5+ 4.0* 4.0  , M_PI},
    {100.5 ,  2.5+ 5.0* 4.0  , M_PI},
    {100.5 ,  2.5+ 6.0* 4.0  , M_PI},
    {100.5 ,  2.5+ 7.0* 4.0  , M_PI},
    {100.5 ,  2.5+ 8.0* 4.0  , M_PI},
    {100.5 ,  2.5+ 9.0* 4.0  , M_PI},
    {100.5  , 2.5+ 10.0*4.0 , M_PI},
    {100.5  , 2.5+ 11.0*4.0 , M_PI},
    {100.5  , 2.5+ 12.0*4.0 , M_PI},
    {100.5  , 2.5+ 13.0*4.0 , M_PI},
    {100.5  , 2.5+ 14.0*4.0 , M_PI},
    {100.5  , 2.5+ 15.0*4.0 , M_PI},
    {100.5  , 2.5+ 16.0*4.0 , M_PI},
    {100.5  , 2.5+ 17.0*4.0 , M_PI},
    {100.5 ,  2.5+ 18.0*4.0 , M_PI},
    {100.5 ,  2.5+ 19.0*4.0 , M_PI},
    {100.5 ,  2.5+ 20.0*4.0 , M_PI},
    {100.5 ,  2.5+ 21.0*4.0 , M_PI},
    {100.5 ,  2.5+ 22.0*4.0 , M_PI},
    {100.5 ,  2.5+ 23.0*4.0 , M_PI},
    {100.5 ,  2.5+ 24.0*4.0 , M_PI},
    {100.5 ,  2.5+ 25.0*4.0 , M_PI},
    {100.5 ,  2.5+ 26.0*4.0 , M_PI},

    {63 ,  84 , 0.75*M_PI+0.0001},
    {59 ,  81 , 0.75*M_PI+0.0001},
    {55 ,  78 , 0.75*M_PI+0.0001},

    {57.5 ,  101.5 ,0.25*M_PI},
    {42.5 ,  95.5  ,0*M_PI},
    {27.5 ,  102   ,0.75*M_PI+0.0001},

    {21.5 ,  70.5 ,0.0},
    {7.5 ,   70.5 ,0.0},
    {21.5 ,  77.5 ,0.0},
    {7.5 ,   77.5 ,0.0}
    };
    return value;
}

inline const std::vector<double>& onsite_radom_length() {
    static const std::vector<double> value = {
        10, 20, 30, 40, 50, 60, 70, 80,
        90, 100, 110, 120, 130, 140, 150,
        160, 170, 180, 190, 200, 210, 220, 230,
         240, 250, 260, 270, 280
    };
    return value ;
}
static const int pairs_per_radius = 20;

static const float cellSize = 1;
static const  int sample_density=2;
static const  int num_sample_points=6 * std::pow(sample_density, 2) + 2;

static const int bbSize = std::ceil((sqrt(width() * width() + length()* length()) + 4) / cellSize);

static const int positionResolution = 6;

static const int positions = positionResolution * positionResolution;

static const int headings = int((2 * M_PI / (real_map  * w_max_wheel)* positionResolution * V_max_vision)/8.0)*8.0;

static const float deltaHeadingDeg = 360 / (float)headings;

static const float deltaHeadingRad = 2 * M_PI / (float)headings;

static const float deltaHeadingNegRad = 2 * M_PI - deltaHeadingRad;

static const float operator_xy = 1.0/positionResolution * 2.0 / (float)sample_density*sample_ratio;
static const float operator_yaw = deltaHeadingRad*(2.0f/(float)sample_density)*sample_ratio;
static const float operator_Negyaw = 2 * M_PI -operator_yaw;

static const float tieBreaker = 0.01;

static const float factor2D = sqrt(5) / sqrt(2) + 1;

static const float penaltyTurning = 1.05;

static const float penaltyReversing = 2.0;

static const float penaltyCOD = 2.0;

static const float dubinsShotDistance = 100;

static const float dubinsStepSize = 1;

static const int dubinsWidth = 15;

static const int dubinsArea = dubinsWidth * dubinsWidth;

struct relPos {

  int x;

  int y;

  relPos() : x(0), y(0) {}
  relPos(int x_, int y_) : x(x_), y(y_) {}
};

struct config {

  int length;

  relPos* pos;
  int max_size;

  config() : length(0), max_size(1000) {
    pos = new relPos[max_size];

    for (int i = 0; i < max_size; ++i) {
      pos[i] = relPos();
    }
  }

  ~config() {
    if (pos) {
      delete[] pos;
      pos = nullptr;
    }
  }

  config(const config& other) : length(other.length), max_size(other.max_size) {
    pos = new relPos[max_size];
    for (int i = 0; i < max_size; ++i) {
      pos[i] = other.pos[i];
    }
  }

  config& operator=(const config& other) {
    if (this != &other) {

      delete[] pos;

      length = other.length;
      max_size = other.max_size;
      pos = new relPos[max_size];
      for (int i = 0; i < max_size; ++i) {
        pos[i] = other.pos[i];
      }
    }
    return *this;
  }

  relPos& operator[](int index) {
    #ifdef DEBUG
    if (index < 0 || index >= max_size) {
      std::cerr << "Error: Index " << index << " out of range in config::pos (max_size: " << max_size << ")" << std::endl;

      static relPos default_pos;
      return default_pos;
    }
    #endif
    return pos[index];
  }

  const relPos& operator[](int index) const {
    #ifdef DEBUG
    if (index < 0 || index >= max_size) {
      std::cerr << "Error: Index " << index << " out of range in config::pos (max_size: " << max_size << ")" << std::endl;

      static relPos default_pos;
      return default_pos;
    }
    #endif
    return pos[index];
  }
};

static const float minRoadWidth = 2;

struct color {

  float red;

  float green;

  float blue;
};

static constexpr color teal = {102.f / 255.f, 217.f / 255.f, 239.f / 255.f};

static constexpr color green = {166.f / 255.f, 226.f / 255.f, 46.f / 255.f};

static constexpr color orange = {253.f / 255.f, 151.f / 255.f, 31.f / 255.f};

static constexpr color pink = {249.f / 255.f, 38.f / 255.f, 114.f / 255.f};

static constexpr color purple = {174.f / 255.f, 129.f / 255.f, 255.f / 255.f};

static constexpr color black = {0.f / 255.f, 0.f / 255.f, 0.f / 255.f};

static constexpr color red = {139.f / 255.f, 0.f / 255.f, 0.f / 255.f};

struct Pathpair {
    bool result_valid;

    double start_x, start_y, start_yaw;
    double goal_x, goal_y, goal_yaw;

    double path_length;
    double path_cost;
    std::vector<std::tuple<double, double, double>> path_xyt;

    double t_judge_output_Astar =0.0;
    double t_judge_output_DSU   =0.0;
    double t_planning_output    =0.0;
    int planning_iterations = 0;

    bool path_is_feasible_Astar    = false;
    bool path_is_feasible_DSU      = false;
    bool path_is_feasible_searching= false;

    int radi_test;

    int length_t;

    Pathpair()
        : result_valid(false),
          start_x(0), start_y(0), start_yaw(0),
          goal_x(0), goal_y(0), goal_yaw(0),
          path_length(0), path_cost(0),
          t_judge_output_Astar(0), t_judge_output_DSU(0), t_planning_output(0), planning_iterations(0),
          path_is_feasible_Astar(false),path_is_feasible_DSU(false),path_is_feasible_searching(false),
          radi_test(0),length_t(0) {}
};

}
}

#endif
