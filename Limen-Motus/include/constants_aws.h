#ifndef CONSTANTS_AWS
#define CONSTANTS_AWS

#include <cmath>
#include <vector>
#include <spmt_model.h>

namespace HybridAStar {

namespace Constants_aws {

static const bool use_aws = true;
static const bool online_test = true;
static const bool time_measurement = false || use_aws;

static const bool coutDEBUG = false;

static const bool manual = true;

static const bool visualization = true && manual;

static const bool visualization2D = false && manual;

static const bool reverse = true && !use_aws;

static const bool dubinsShot = true;

static const bool dubins = false;

static const bool path_smooth = false;

static const bool dubinsLookup = false && dubins;

static const bool twoD = false;
static const bool eula_dis = true && !twoD;
static const bool heading_heuristic = true;

static const int iterations = 150000000/259;

inline const double& bloating() {
    static const double value = 0.55;
    return value;
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

static const bool outline_collision_avoid_test = false;
static const bool outline_collision_avoid_cost = false;
static const bool online_outline_collision_avoid = true;
static const int wheel_num = 12;

static const float r = 6;
static const float g_range = 1.5;

static const int headings = 16;

static const float deltaHeadingDeg = 360 / (float)headings;

static const float deltaHeadingRad = 2 * M_PI / (float)headings;

static const float cellSize = 1;

static const float wheel_base = 1.4;
static const float wheel_width = 1.45;
static const float steering_limit = M_PI/180*90;

static const float half_l = wheel_base/2.0;
static const float half_w = wheel_width/2.0;
static const bool has_steer_limit = true;

static const int psi_sample_num = 8;
static const int omega_sample_num = 8;
static const int r_sample_num = 8;
static const float control_center_offset = -half_l;

static const std::vector<float> wheel_positions_x=
  {

  half_l*5.0,
  half_l*5.0,

  half_l*3.0,
  half_l*3.0,

  half_l*1.0,
  half_l*1.0,

  half_l*(-1.0),
  half_l*(-1.0),

  half_l*(-3.0),
  half_l*(-3.0),

  half_l*(-5.0),
  half_l*(-5.0)
};
static const std::vector<float> wheel_positions_y=
  {

  half_w*(1.0),
  half_w*(-1.0),

  half_w*(1.0),
  half_w*(-1.0),

  half_w*(1.0),
  half_w*(-1.0),

  half_w*(1.0),
  half_w*(-1.0),

  half_w*(1.0),
  half_w*(-1.0),

  half_w*(1.0),
  half_w*(-1.0)
};
static const float rear_steer_limit = steering_limit;

static const std::vector<float> wheel_steer_limits_up =
  {
    steering_limit, steering_limit, rear_steer_limit, rear_steer_limit,
    steering_limit, steering_limit, rear_steer_limit, rear_steer_limit,
    steering_limit, steering_limit, rear_steer_limit, rear_steer_limit,
    steering_limit, steering_limit, rear_steer_limit, rear_steer_limit
  };
static const std::vector<float> wheel_steer_limits_low =
  {
    -steering_limit, -steering_limit, -rear_steer_limit, -rear_steer_limit,
    -steering_limit, -steering_limit, -rear_steer_limit, -rear_steer_limit,
    -steering_limit, -steering_limit, -rear_steer_limit, -rear_steer_limit,
    -steering_limit, -steering_limit, -rear_steer_limit, -rear_steer_limit
  };
static const float max_steering_v = M_PI/3;
static const float max_wheel_v = 2.5;
static const float max_wheel_a = 2.5;

static const float tieBreaker = 0.01;

static const float factor2D = sqrt(5) / sqrt(2) + 1;

static const float penaltyTurning = 1.05;

static const float penaltyReversing = 2.0;

static const float penaltyCOD = 2.0;

static const float dubinsShotDistance = 3;

static const float dubinsStepSize = 1;

static const int dubinsWidth = 15;

static const int dubinsArea = dubinsWidth * dubinsWidth;

static const int bbSize = std::ceil((sqrt(width() * width() + length() * length()) + 4) / cellSize);

static const int positionResolution = 6;

static const int positions = positionResolution * positionResolution;

struct relPos {

  int x;

  int y;
};

struct config {

  int length;

  relPos pos[64];
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
}
}

#endif
