#include "node_aws.h"
#include <algorithm>
#include <iostream>

using namespace HybridAStar;

bool NodeAWS::isInRange(const NodeAWS& goal) const {
  return std::abs(getX() - goal.getX()) < Constants_aws::cellSize &&
         std::abs(getY() - goal.getY()) < Constants_aws::cellSize &&
         std::abs(Helper::normalizeHeadingRad(getT() - goal.getT())) < Constants_aws::deltaHeadingRad;
}

NodeAWS* NodeAWS::createSuccessor(const int i) {
  float xSucc;
  float ySucc;
  float tSucc;

  std::vector<float> vi = sampling_table->at(i);
  xSucc = getX() + vi[0] * cos(getT()) - vi[1] * sin(getT());
  ySucc = getY() + vi[0] * sin(getT()) + vi[1] * cos(getT());
  tSucc = Helper::normalizeHeadingRad(getT() + vi[2]);

  return new NodeAWS(xSucc, ySucc, tSucc, getG(), 0.,
                    vi[3], vi[4], vi[5], i,
                    this, sampling_table);
}

NodeAWS::WheelCtrlCmd NodeAWS::RelativeVAndOmegaControl(double vx, double vy, double omega, double lim_angle, bool wheel_rotate_velocity) {

    auto wheel_positions_x=Constants_aws::wheel_positions_x;

    auto wheel_positions_y=Constants_aws::wheel_positions_y;

    WheelCtrlCmd wheel_ctrl_cmd;

    for (int i = 0; i < (int)wheel_positions_x.size(); ++i) {
      auto wheel_vx = vx - wheel_positions_y.at(i) * omega;
      auto wheel_vy = vy + wheel_positions_x.at(i) * omega;
      double wheel_v = sqrt(wheel_vx * wheel_vx + wheel_vy * wheel_vy);
      double wheel_s = atan2(wheel_vy, wheel_vx);

      if (Constants_aws::has_steer_limit){
      float limit_angle_u = Constants_aws::wheel_steer_limits_up[i];
      float limit_angle_l = Constants_aws::wheel_steer_limits_low[i];
      bool oob_wheel_ub = (wheel_s > limit_angle_u)
                       && (Helper::normalizeHeadingRad(wheel_s+M_PI)
                       > limit_angle_l);
      bool oob_wheel_lb = (wheel_s < limit_angle_l)
                       && (Helper::normalizeHeadingRad(wheel_s-M_PI)
                       < limit_angle_u);
      bool oob_wheel = oob_wheel_ub || oob_wheel_lb;
      if (oob_wheel) {
        wheel_s = Helper::normalizeHeadingRad(wheel_s + M_PI);
        wheel_v = -wheel_v;
      }
      }

      wheel_ctrl_cmd.first.push_back(wheel_v);
      wheel_ctrl_cmd.second.push_back(wheel_s);
    }

    return wheel_ctrl_cmd;
}

void NodeAWS::updateG() {

  double wholebody_v_t = sqrt(pow(getVX(),2)+pow(getVY(),2)) \
                         / Constants_aws::max_wheel_v;
  double wholebody_w_t = fabs(getVYaw()/Constants_aws::max_steering_v);
  double wb_cost = std::max(wholebody_v_t, wholebody_w_t);
  if (getPred() == nullptr) {
    getG_mutable() += wb_cost;
    return;
  }
  auto pred_ctrl = RelativeVAndOmegaControl(
    getPred()->getVX(), getPred()->getVY(), getPred()->getVYaw());
  auto succ_ctrl = RelativeVAndOmegaControl(
    getVX(), getVY(), getVYaw());
  std::vector<double> diff_wheel_speed;
  std::vector<double> diff_wheel_pose;
  std::vector<double> maneuver_time;

  for (int i = 0; i < (int)pred_ctrl.first.size(); ++i) {
    diff_wheel_speed.push_back(
      std::abs(- pred_ctrl.first.at(i) + succ_ctrl.first.at(i))
      /Constants_aws::max_wheel_a
      );
    diff_wheel_pose.push_back(
      std::abs(- pred_ctrl.second.at(i) + succ_ctrl.second.at(i))
      /Constants_aws::max_steering_v
      );
    maneuver_time.push_back(
      pow(
        pow(diff_wheel_speed.at(i),2) + pow(diff_wheel_pose.at(i),2),
        0.5)
      );
  }

  double max_wheel_pose_diff = *std::max_element(
                diff_wheel_pose.begin(), diff_wheel_pose.end());
  double max_wheel_speed_diff = *std::max_element(
                diff_wheel_speed.begin(), diff_wheel_speed.end());
  double max_wheel_t = *std::max_element(
                maneuver_time.begin(), maneuver_time.end());

  getG_mutable() = getG() + std::max(wb_cost, max_wheel_t);

}
