#include <ros/ros.h>

#include "planner.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "limen_motus");
  HybridAStar::Planner planner;
  planner.plan();
  ros::spin();
  return 0;
}
