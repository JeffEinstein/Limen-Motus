#ifndef VISUALIZE_H
#define VISUALIZE_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "gradient.h"

#include "node3d.h"
#include "node2d.h"
namespace HybridAStar {
class Node3D;
class Node2D;

class Visualize {
 public:

  Visualize() {

    pubNode3D = n.advertise<geometry_msgs::PoseStamped>("/visualizeNodes3DPose", 100);
    pubNodes3D = n.advertise<geometry_msgs::PoseArray>("/visualizeNodes3DPoses", 100);
    pubNodes3Dreverse = n.advertise<geometry_msgs::PoseArray>("/visualizeNodes3DPosesReverse", 100);
    pubNodes3DCosts = n.advertise<visualization_msgs::MarkerArray>("/visualizeNodes3DCosts", 100);
    pubNode2D = n.advertise<geometry_msgs::PoseStamped>("/visualizeNodes2DPose", 100);
    pubNodes2D = n.advertise<geometry_msgs::PoseArray>("/visualizeNodes2DPoses", 100);
    pubNodes2DCosts = n.advertise<visualization_msgs::MarkerArray>("/visualizeNodes2DCosts", 100);

    poses3D.header.frame_id = "path";
    poses3Dreverse.header.frame_id = "path";
    poses2D.header.frame_id = "path";
  }

  void clear();

  void clear2D() {poses2D.poses.clear();}

  void publishNode3DPose(Node3D& node);

  void publishNode3DPoses(Node3D& node);

  void publishNode3DCosts(Node3D* nodes, int width, int height, int depth);

  void publishNode2DPose(Node2D& node);

  void publishNode2DPoses(Node2D& node);

  void publishNode2DCosts(Node2D* nodes, int width, int height);

 private:

  ros::NodeHandle n;

  ros::Publisher pubNode3D;

  ros::Publisher pubNodes3D;

  ros::Publisher pubNodes3Dreverse;

  ros::Publisher pubNodes3DCosts;

  ros::Publisher pubNode2D;

  ros::Publisher pubNodes2D;

  ros::Publisher pubNodes2DCosts;

  geometry_msgs::PoseArray poses3D;

  geometry_msgs::PoseArray poses3Dreverse;

  geometry_msgs::PoseArray poses2D;

};
}
#endif
