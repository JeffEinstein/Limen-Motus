#include "visualize.h"
using namespace HybridAStar;

void Visualize::clear() {
  poses3D.poses.clear();
  poses3Dreverse.poses.clear();
  poses2D.poses.clear();

  visualization_msgs::MarkerArray costCubes3D;
  visualization_msgs::Marker costCube3D;

  costCube3D.header.frame_id = "path";
  costCube3D.header.stamp = ros::Time::now();
  costCube3D.id = 0;
  costCube3D.action = 3;
  costCubes3D.markers.push_back(costCube3D);
  pubNodes3DCosts.publish(costCubes3D);

  visualization_msgs::MarkerArray costCubes2D;
  visualization_msgs::Marker costCube2D;

  costCube2D.header.frame_id = "path";
  costCube2D.header.stamp = ros::Time::now();
  costCube2D.id = 0;
  costCube2D.action = 3;
  costCubes2D.markers.push_back(costCube2D);
  pubNodes2DCosts.publish(costCubes2D);
}

void Visualize::publishNode3DPose(Node3D& node) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "path";
  pose.header.stamp = ros::Time::now();
  pose.header.seq = 0;
  pose.pose.position.x = node.getX() * Constants::cellSize;
  pose.pose.position.y = node.getY() * Constants::cellSize;

  if (node.getPrim() < 3) {
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
  }

  else {
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT() + M_PI);
  }

  pubNode3D.publish(pose);
}

void Visualize::publishNode3DPoses(Node3D& node) {
  geometry_msgs::Pose pose;
  pose.position.x = node.getX() * Constants::cellSize;
  pose.position.y = node.getY() * Constants::cellSize;

  if (node.getPrim() < 3) {
    pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
    poses3D.poses.push_back(pose);
    poses3D.header.stamp = ros::Time::now();

    pubNodes3D.publish(poses3D);
  }

  else {
    pose.orientation = tf::createQuaternionMsgFromYaw(node.getT() + M_PI);
    poses3Dreverse.poses.push_back(pose);
    poses3Dreverse.header.stamp = ros::Time::now();

    pubNodes3Dreverse.publish(poses3Dreverse);
  }

}

void Visualize::publishNode2DPose(Node2D& node) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "path";
  pose.header.stamp = ros::Time::now();
  pose.header.seq = 0;
  pose.pose.position.x = (node.getX() + 0.5) * Constants::cellSize;
  pose.pose.position.y = (node.getY() + 0.5) * Constants::cellSize;
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

  pubNode2D.publish(pose);
}

void Visualize::publishNode2DPoses(Node2D& node) {
  if (node.isDiscovered()) {
    geometry_msgs::Pose pose;
    pose.position.x = (node.getX() + 0.5) * Constants::cellSize;
    pose.position.y = (node.getY() + 0.5) * Constants::cellSize;
    pose.orientation = tf::createQuaternionMsgFromYaw(0);

    poses2D.poses.push_back(pose);
    poses2D.header.stamp = ros::Time::now();

    pubNodes2D.publish(poses2D);

  }
}

void Visualize::publishNode3DCosts(Node3D* nodes, int width, int height, int depth) {
  visualization_msgs::MarkerArray costCubes;
  visualization_msgs::Marker costCube;

  float min = 1000;
  float max = 0;
  int idx;
  bool once = true;
  float red = 0;
  float green = 0;
  float blue = 0;
  int count = 0;

  ColorGradient heatMapGradient;
  heatMapGradient.createDefaultHeatMapGradient();

  float values[width * height];

  for (int i = 0; i < width * height; ++i) {
    values[i] = 1000;

    for (int k = 0; k < depth; ++k) {
      idx = k * width * height + i;

      if (nodes[idx].isClosed() || nodes[idx].isOpen()) {
        values[i] = nodes[idx].getC();
      }
    }

    if (values[i] > 0 && values[i] < min) {
      min = values[i];
    }

    if (values[i] > 0 && values[i] > max && values[i] != 1000) {
      max = values[i];
    }
  }

  for (int i = 0; i < width * height; ++i) {

    if (values[i] != 1000) {
      count++;

      if (once) {
        costCube.action = 3;
        once = false;
      } else {
        costCube.action = 0;
      }

      costCube.header.frame_id = "path";
      costCube.header.stamp = ros::Time::now();
      costCube.id = i;
      costCube.type = visualization_msgs::Marker::CUBE;
      values[i] = (values[i] - min) / (max - min);
      costCube.scale.x = Constants::cellSize;
      costCube.scale.y = Constants::cellSize;
      costCube.scale.z = 0.1;
      costCube.color.a = 0.5;
      heatMapGradient.getColorAtValue(values[i], red, green, blue);
      costCube.color.r = red;
      costCube.color.g = green;
      costCube.color.b = blue;

      costCube.pose.position.x = (i % width + 0.5) * Constants::cellSize;
      costCube.pose.position.y = ((i / width) % height + 0.5) * Constants::cellSize;
      costCubes.markers.push_back(costCube);
    }
  }

  if (Constants::coutDEBUG) {
    std::cout << "3D min cost: " << min << " | max cost: " << max << std::endl;
    std::cout << count << " 3D nodes expanded " << std::endl;
  }

  pubNodes3DCosts.publish(costCubes);
}

void Visualize::publishNode2DCosts(Node2D* nodes, int width, int height) {
  visualization_msgs::MarkerArray costCubes;
  visualization_msgs::Marker costCube;

  float min = 1000;
  float max = 0;
  bool once = true;
  float red = 0;
  float green = 0;
  float blue = 0;
  int count = 0;

  ColorGradient heatMapGradient;
  heatMapGradient.createDefaultHeatMapGradient();

  float values[width * height];

  for (int i = 0; i < width * height; ++i) {
    values[i] = 1000;

    if (nodes[i].isDiscovered()) {
      values[i] = nodes[i].getG();

      if (values[i] > 0 && values[i] < min) { min = values[i]; }

      if (values[i] > 0 && values[i] > max) { max = values[i]; }
    }
  }

  for (int i = 0; i < width * height; ++i) {

    if (nodes[i].isDiscovered()) {
      count++;

      if (once) {
        costCube.action = 3;
        once = false;
      } else {
        costCube.action = 0;
      }

      costCube.header.frame_id = "path";
      costCube.header.stamp = ros::Time::now();
      costCube.id = i;
      costCube.type = visualization_msgs::Marker::CUBE;
      values[i] = (values[i] - min) / (max - min);
      costCube.scale.x = Constants::cellSize;
      costCube.scale.y = Constants::cellSize;
      costCube.scale.z = 0.1;
      costCube.color.a = 0.5;
      heatMapGradient.getColorAtValue(values[i], red, green, blue);
      costCube.color.r = red;
      costCube.color.g = green;
      costCube.color.b = blue;

      costCube.pose.position.x = (i % width + 0.5) * Constants::cellSize;
      costCube.pose.position.y = ((i / width) % height + 0.5) * Constants::cellSize;
      costCubes.markers.push_back(costCube);
    }
  }

  if (Constants::coutDEBUG) {
    std::cout << "2D min cost: " << min << " | max cost: " << max << std::endl;
    std::cout << count << " 2D nodes expanded " << std::endl;
  }

  pubNodes2DCosts.publish(costCubes);
}
