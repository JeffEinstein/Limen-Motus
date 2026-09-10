#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "constants.h"
#include "helper.h"
#include "collisiondetection.h"
#include "dynamicvoronoi.h"
#include "algorithm.h"
#include "node3d.h"
#include "path.h"
#include "visualize.h"
#include "lookup.h"
#include "vplanner.h"
#include "show.h"

namespace HybridAStar {

class Planner {
 public:

  Planner();

  void initializeLookups();

  void setMap(const nav_msgs::OccupancyGrid::Ptr map);

  void setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start);

  void setGoal(const geometry_msgs::PoseStamped::ConstPtr& goal);

  void plan();

   Constants::Pathpair sg_pair[560];

   int find_sg_pair(Constants::Pathpair* pair, int width, int height, bool enable_visualization = false);
   int sg_pair_finded=false;
   int num_pairs=0;

 private:

  ros::NodeHandle n;

  ros::Publisher pubStart;
  ros::Publisher pubGoal;

  ros::Subscriber subMap;

  ros::Subscriber subGoal;

  ros::Subscriber subStart;

  tf::TransformListener listener;

  tf::StampedTransform transform;

  Path path;


  VelocityPlanner vplanner;

  Visualize visualization;

  CollisionDetection configurationSpace;

  DynamicVoronoi voronoiDiagram;

  nav_msgs::OccupancyGrid::Ptr grid;

  geometry_msgs::PoseWithCovarianceStamped start;

  geometry_msgs::PoseStamped goal;

  bool validStart = false;

  bool validGoal = false;

  Constants::config collisionLookup[Constants::headings * Constants::positions];

  float* dubinsLookup = new float [Constants::headings * Constants::headings * Constants::dubinsWidth * Constants::dubinsWidth];

  std::vector<int> dsuParent;
  std::vector<int> dsuRank;
  bool dsuInitialized = false;
  int dsuWidth = 0;
  int dsuHeight = 0;
  std::string dsuMapFrameId;

};
}
#endif
