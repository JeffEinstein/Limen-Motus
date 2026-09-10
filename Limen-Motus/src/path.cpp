#include "path.h"

using namespace HybridAStar;

std::vector<std::vector<int>> Path::precomputedCarShapeTriangles;
bool Path::trianglesInitialized = false;

void Path::clear() {
  Node3D node;
  path.poses.clear();
  pathNodes.markers.clear();
  pathVehicles.markers.clear();
  addNode(node, 0);
  addVehicle_SPMT(node, 1, true);
  publishPath();
  publishPathNodes();
  publishPathVehicles();
}
void Path::clear_vehicle() {
  Node3D node;
  pathVehicles.markers.clear();
  addVehicle_SPMT(node, 1,true);
  publishPathVehicles();
}

void Path::updatePath(const std::vector<Node3D>& nodePath) {
  path.header.stamp = ros::Time::now();
  int k = 0;

  for (size_t i = 0; i < nodePath.size(); ++i) {
    addSegment(nodePath[i], "onebyone");
    addNode(nodePath[i], k, "onebyone");
    k++;
    addVehicle_SPMT_earcut(nodePath[i], k,true,false);
    k++;

  }

  return;
}

void Path::updatePath_onlynode(const std::vector<Node3D>& nodePath) {
  path.header.stamp = ros::Time::now();
  int k = 0;

  path.poses.clear();

  for (size_t i = 0; i < nodePath.size(); ++i) {
    addSegment(nodePath[i],"");

    k++;

  }
  publishPath();
  return;
}

void Path::updatePath_v(const std::vector<Node3D> &nodePath,
const std::vector<VelocityPoint> velocity_profile) {
  path.header.stamp = ros::Time::now();
  int k = 0;

  for (size_t i = 0; i < nodePath.size(); ++i) {
    clear_vehicle();
    addSegment(nodePath[nodePath.size()-i-1], "onebyone");
    addNode(nodePath[nodePath.size()-i-1], k, "onebyone");
    k++;
    addVehicle(nodePath[nodePath.size()-i-1], k, "onebyone");
    k++;
    if(i>nodePath.size()-3)continue;
    double dur=double(velocity_profile[nodePath.size()-i].time_after_before);
    ros::Duration(dur).sleep();
  }

   return;
}

int Path::updatePathWithColor(const std::vector<Node3D>& nodePath, PathColor color,int k) {
  path.header.stamp = ros::Time::now();
  for (size_t i = 0; i < nodePath.size(); ++i) {
    addSegment(nodePath[i], "");
    k++;
  }
  publishPath();
  return k+1;
}

void Path::addSegment(const Node3D& node, std::string visaul_choose) {
  geometry_msgs::PoseStamped vertex;
  vertex.pose.position.x = node.getX() * Constants::cellSize;
  vertex.pose.position.y = node.getY() * Constants::cellSize;
  vertex.pose.position.z = 0;
  vertex.pose.orientation.x = 0;
  vertex.pose.orientation.y = 0;
  vertex.pose.orientation.z = 0;
  vertex.pose.orientation.w = 0;
  if(visaul_choose=="onebyone"){
    path.poses.clear();
    path.poses.push_back(vertex);
    pubPath.publish(path);
  }else
  path.poses.push_back(vertex);

}

void Path::addNode(const Node3D& node, int i, std::string visaul_choose) {
  visualization_msgs::Marker pathNode;

  if (i == -1) {
    pathNode.action = 3;
  }

  pathNode.header.frame_id = "path";
  pathNode.header.stamp = ros::Time(0);
  pathNode.id = i;
  pathNode.type = visualization_msgs::Marker::SPHERE;
  pathNode.scale.x = 0.1;
  pathNode.scale.y = 0.1;
  pathNode.scale.z = 0.1;
  pathNode.color.a = 1.0;

  if (smoothed) {
    pathNode.color.r = Constants::black.red;
    pathNode.color.g = Constants::black.green;
    pathNode.color.b = Constants::black.blue;
  } else {
    pathNode.color.r = Constants::purple.red;
    pathNode.color.g = Constants::purple.green;
    pathNode.color.b = Constants::purple.blue;
  }

  pathNode.pose.position.x = node.getX() * Constants::cellSize;
  pathNode.pose.position.y = node.getY() * Constants::cellSize;
  if(visaul_choose=="onebyone"){
    pathNodes.markers.clear();
    pathNodes.markers.push_back(pathNode);
    pubPathNodes.publish(pathNodes);
}else
  pathNodes.markers.push_back(pathNode);
}

void Path::addNode_bigger(const Node3D& node, int i, std::string visaul_choose) {
  visualization_msgs::Marker pathNode;

  if (i == -1) {
    pathNode.action = 3;
  }

  pathNode.header.frame_id = "path";
  pathNode.header.stamp = ros::Time(0);
  pathNode.id = i;
  pathNode.type = visualization_msgs::Marker::SPHERE;
  pathNode.scale.x = 1;
  pathNode.scale.y = 1;
  pathNode.scale.z = 1;
  pathNode.color.a = 1.0;

  if (smoothed) {
    pathNode.color.r = Constants::black.red;
    pathNode.color.g = Constants::black.green;
    pathNode.color.b = Constants::black.blue;
  } else {
    pathNode.color.r = Constants::purple.red;
    pathNode.color.g = Constants::purple.green;
    pathNode.color.b = Constants::purple.blue;
  }

  pathNode.pose.position.x = node.getX() * Constants::cellSize;
  pathNode.pose.position.y = node.getY() * Constants::cellSize;
  if(visaul_choose=="onebyone"){
    pathNodes.markers.clear();
    pathNodes.markers.push_back(pathNode);
    pubPathNodes.publish(pathNodes);
}else
  pathNodes.markers.push_back(pathNode);
}

void Path::addVehicle(const Node3D& node, int i, std::string visaul_choose) {
  visualization_msgs::Marker pathVehicle;

  if (i == 1) {
    pathVehicle.action = 3;
  }else{  pathVehicle.action = 0; }

  pathVehicle.header.frame_id = "map";
  pathVehicle.header.stamp = ros::Time(0);
  pathVehicle.id = i;
  pathVehicle.type = visualization_msgs::Marker::CUBE;
  pathVehicle.scale.x = Constants::length() - Constants::bloating() * 2.0;
  pathVehicle.scale.y = Constants::width() - Constants::bloating() * 2.0;
  pathVehicle.scale.z = 1;
  pathVehicle.color.a = 0.1;
  pathVehicle.ns = "spmt_car_shape";

  if (smoothed) {
    pathVehicle.color.r = Constants::red.red;
    pathVehicle.color.g = Constants::red.green;
    pathVehicle.color.b = Constants::red.blue;
  } else {
    pathVehicle.color.r = Constants::teal.red;
    pathVehicle.color.g = Constants::teal.green;
    pathVehicle.color.b = Constants::teal.blue;
  }

  pathVehicle.pose.position.x = node.getX() * Constants::cellSize;
  pathVehicle.pose.position.y = node.getY() * Constants::cellSize;
  pathVehicle.pose.position.z = 0.5;
  pathVehicle.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
  if(visaul_choose=="onebyone"){

    pathVehicles.markers.push_back(pathVehicle);
    pubPathVehicles.publish(pathVehicles);
  }else
  pathVehicles.markers.push_back(pathVehicle);

  if(i == 1){
    pubPathVehicles.publish(pathVehicles);
  }
}

void Path::addVehicle_SPMT(const Node3D& node, int i, bool clear) {

  const std::vector<Eigen::Vector2d>& local_shape = HybridAStar::Constants::my_car_shape_for_show();

  if (local_shape.size() < 3) {
    return;
  }

  visualization_msgs::Marker pathVehicle;

  if(i==1){
    pathVehicle.action = 3;
  }else{
    pathVehicle.action = visualization_msgs::Marker::ADD;
  }

  pathVehicle.header.frame_id = "map";
  pathVehicle.header.stamp = ros::Time(0);
  pathVehicle.id = i;

  pathVehicle.ns = "spmt_car_shape";

  pathVehicle.type = visualization_msgs::Marker::TRIANGLE_LIST;

  pathVehicle.color.a = 0.7;
  pathVehicle.color.r = 1.0;
  pathVehicle.color.g = 0.0;
  pathVehicle.color.b = 0.0;

  pathVehicle.scale.x = 1.0;
  pathVehicle.scale.y = 1.0;
  pathVehicle.scale.z = 1.0;

  double node_x = node.getX() * Constants::cellSize;
  double node_y = node.getY() * Constants::cellSize;
  double node_yaw = node.getT();

  Eigen::Rotation2Dd rotation(node_yaw);
  Eigen::Vector2d translation(node_x, node_y);

  Eigen::Vector2d p0_global = rotation * local_shape[0] + translation;

  for (size_t j = 1; j < local_shape.size() - 1; ++j) {

    Eigen::Vector2d p1_global = rotation * local_shape[j] + translation;
    Eigen::Vector2d p2_global = rotation * local_shape[j + 1] + translation;

    geometry_msgs::Point p0, p1, p2;

    p0.x = p0_global.x(); p0.y = p0_global.y(); p0.z = 0.5;
    p1.x = p1_global.x(); p1.y = p1_global.y(); p1.z = 0.5;
    p2.x = p2_global.x(); p2.y = p2_global.y(); p2.z = 0.5;

    pathVehicle.points.push_back(p0);
    pathVehicle.points.push_back(p1);
    pathVehicle.points.push_back(p2);
  }

  if(clear){
    Node3D nodeclear;
    addVehicle(nodeclear,1);
  }

  pathVehicles.markers.push_back(pathVehicle);

  pubPathVehicles.publish(pathVehicles);

  ros::Duration(0.5).sleep();

}

void Path::addVehicle_SPMT_earcut(const Node3D& node, int i, bool clear_body,bool clear_outline, PathColor color) {

  const std::vector<Eigen::Vector2d>& local_shape = HybridAStar::Constants::my_car_shape_for_show();
        int positionResolution;
    int headings;
    double deltaHeadingRad;
    double cell_size;

    if(Constants::Algorithm_Framework=="AWS_from_others"){
      positionResolution = Constants::positionResolution;
      headings = Constants::headings;
      deltaHeadingRad = Constants::deltaHeadingRad;
      cell_size = Constants::cellSize;
    }else{
      positionResolution = Constants_aws::positionResolution;
      headings = Constants_aws::headings;
      deltaHeadingRad = Constants_aws::deltaHeadingRad;
      cell_size = Constants_aws::cellSize;
    }

  float x=node.getX();
  float y=node.getY();
  float t=node.getT();
  int X = (int)x;
  int Y = (int)y;
  int iX = (int)((x - (long)x) * positionResolution);
  iX = iX > 0 ? iX : 0;
  int iY = (int)((y - (long)y) * positionResolution);
  iY = iY > 0 ? iY : 0;
  int iT = (int)(t / deltaHeadingRad);

  x = X+iX*1.0f/positionResolution;
  y = Y+iY*1.0f/positionResolution;
  t = iT*deltaHeadingRad;

  Node3D nStart(x, y, t, 0, 0, nullptr);

  addVehicleOutlineEdges(nStart, i, color);

  if (local_shape.size() < 3) {
    ROS_WARN("Vehicle shape has less than 3 points, cannot create a polygon.");
    return;
  }

  visualization_msgs::Marker pathVehicle;
  pathVehicle.action = (i == 1) ? visualization_msgs::Marker::DELETEALL : visualization_msgs::Marker::ADD;
  pathVehicle.header.frame_id = "map";
  pathVehicle.header.stamp = ros::Time::now();
  pathVehicle.id = i;
  pathVehicle.ns = "spmt_car_shape_ear";
  pathVehicle.type = visualization_msgs::Marker::TRIANGLE_LIST;
  pathVehicle.color.a = 0.7;
  pathVehicle.color.r = color.r;
  pathVehicle.color.g = color.g;
  pathVehicle.color.b = color.b;
  pathVehicle.scale.x = 1.0;
  pathVehicle.scale.y = 1.0;
  pathVehicle.scale.z = 1.0;

  double node_x = nStart.getX() * Constants::cellSize;
  double node_y = nStart.getY() * Constants::cellSize;
  double node_yaw = nStart.getT();

  Eigen::Rotation2Dd rotation(node_yaw);
  Eigen::Vector2d translation(node_x, node_y);

  std::vector<Eigen::Vector2d> global_shape;
  for (const auto& p_local : local_shape) {
    global_shape.push_back(rotation * p_local + translation);
  }

  if (!trianglesInitialized) {
      const std::vector<Eigen::Vector2d>& car_shape = Constants::my_car_shape_for_show();
      precomputedCarShapeTriangles = earClipping(car_shape);
      trianglesInitialized = true;

      if (precomputedCarShapeTriangles.empty()) {
          ROS_ERROR("Failed to pre-compute car shape triangulation!");
          return;
      }
  }

  const std::vector<std::vector<int>>& triangle_indices = precomputedCarShapeTriangles;

  for (const auto& tri : triangle_indices) {
    geometry_msgs::Point p0, p1, p2;
    p0.x = global_shape[tri[0]].x(); p0.y = global_shape[tri[0]].y(); p0.z = 0.5;
    p1.x = global_shape[tri[1]].x(); p1.y = global_shape[tri[1]].y(); p1.z = 0.5;
    p2.x = global_shape[tri[2]].x(); p2.y = global_shape[tri[2]].y(); p2.z = 0.5;

    pathVehicle.points.push_back(p0);
    pathVehicle.points.push_back(p1);
    pathVehicle.points.push_back(p2);
  }

  if (clear_body) {

    visualization_msgs::Marker clear_marker;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    clear_marker.ns = pathVehicle.ns;
    clear_marker.header.frame_id = "map";
    pathVehicles.markers.push_back(clear_marker);

  }
  if (clear_outline) {

    addVehicleOutlineEdges(nStart, 1,color);
  }
  pathVehicles.markers.push_back(pathVehicle);
  pubPathVehicles.publish(pathVehicles);

  ros::Duration(0.5).sleep();
}

void Path::addVehicle_SPMT_earcut_forever(const Node3D& node, int i, bool clear, PathColor color) {

  const std::vector<Eigen::Vector2d>& local_shape = HybridAStar::Constants::my_car_shape_for_show();

        int positionResolution;
    int headings;
    double deltaHeadingRad;
    double cell_size;

    if(Constants::Algorithm_Framework=="AWS_from_others"){
      positionResolution = Constants::positionResolution;
      headings = Constants::headings;
      deltaHeadingRad = Constants::deltaHeadingRad;
      cell_size = Constants::cellSize;
    }else{
      positionResolution = Constants_aws::positionResolution;
      headings = Constants_aws::headings;
      deltaHeadingRad = Constants_aws::deltaHeadingRad;
      cell_size = Constants_aws::cellSize;
    }

  float x=node.getX();
  float y=node.getY();
  float t=node.getT();
  int X = (int)x;
  int Y = (int)y;
  int iX = (int)((x - (long)x) * positionResolution);
  iX = iX > 0 ? iX : 0;
  int iY = (int)((y - (long)y) * positionResolution);
  iY = iY > 0 ? iY : 0;
  int iT = (int)(t / deltaHeadingRad);

  x = X+iX*1.0f/positionResolution;
  y = Y+iY*1.0f/positionResolution;
  t = iT*deltaHeadingRad;

  Node3D nStart(x, y, t, 0, 0, nullptr);

  addVehicleOutlineEdges(nStart, i,color);

  if (local_shape.size() < 3) {
    ROS_WARN("Vehicle shape has less than 3 points, cannot create a polygon.");
    return;
  }

  visualization_msgs::Marker pathVehicle;
  pathVehicle.action = (i == 1) ? visualization_msgs::Marker::DELETEALL : visualization_msgs::Marker::ADD;
  pathVehicle.header.frame_id = "map";
  pathVehicle.header.stamp = ros::Time::now();
  pathVehicle.id = i;
  pathVehicle.ns = "spmt_car_shape_ear_forever";
  pathVehicle.type = visualization_msgs::Marker::TRIANGLE_LIST;
  pathVehicle.color.a = 0.7;
  pathVehicle.color.r = color.r;
  pathVehicle.color.g = color.g;
  pathVehicle.color.b = color.b;
  pathVehicle.scale.x = 1.0;
  pathVehicle.scale.y = 1.0;
  pathVehicle.scale.z = 1.0;

  double node_x = nStart.getX() * Constants::cellSize;
  double node_y = nStart.getY() * Constants::cellSize;
  double node_yaw = nStart.getT();

  Eigen::Rotation2Dd rotation(node_yaw);
  Eigen::Vector2d translation(node_x, node_y);

  std::vector<Eigen::Vector2d> global_shape;
  for (const auto& p_local : local_shape) {
    global_shape.push_back(rotation * p_local + translation);
  }

  if (!trianglesInitialized) {
      const std::vector<Eigen::Vector2d>& car_shape = Constants::my_car_shape_for_show();
      precomputedCarShapeTriangles = earClipping(car_shape);
      trianglesInitialized = true;

      if (precomputedCarShapeTriangles.empty()) {
          ROS_ERROR("Failed to pre-compute car shape triangulation!");
          return;
      }
  }

  const std::vector<std::vector<int>>& triangle_indices = precomputedCarShapeTriangles;

  for (const auto& tri : triangle_indices) {
    geometry_msgs::Point p0, p1, p2;
    p0.x = global_shape[tri[0]].x(); p0.y = global_shape[tri[0]].y(); p0.z = 0.5;
    p1.x = global_shape[tri[1]].x(); p1.y = global_shape[tri[1]].y(); p1.z = 0.5;
    p2.x = global_shape[tri[2]].x(); p2.y = global_shape[tri[2]].y(); p2.z = 0.5;

    pathVehicle.points.push_back(p0);
    pathVehicle.points.push_back(p1);
    pathVehicle.points.push_back(p2);
  }

  if (clear) {

    visualization_msgs::Marker clear_marker;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    clear_marker.ns = pathVehicle.ns;
    clear_marker.header.frame_id = "map";
    pathVehicles_forever.markers.push_back(clear_marker);
  }

  pathVehicles_forever.markers.push_back(pathVehicle);
  pubPathVehicles_forever.publish(pathVehicles_forever);

  ros::Duration(0.5).sleep();
}

void Path::addVehicle_TriangleMarker(const Node3D& node, int i, bool clear, PathColor color, float scale) {

  visualization_msgs::Marker pathVehicle;
  pathVehicle.action = (i == 1) ? visualization_msgs::Marker::DELETEALL : visualization_msgs::Marker::ADD;
  pathVehicle.header.frame_id = "map";
  pathVehicle.header.stamp = ros::Time::now();
  pathVehicle.id = i;
  pathVehicle.ns = "triangle_marker";
  pathVehicle.type = visualization_msgs::Marker::TRIANGLE_LIST;
  pathVehicle.color.a = color.a;
  pathVehicle.color.r = color.r;
  pathVehicle.color.g = color.g;
  pathVehicle.color.b = color.b;
  pathVehicle.scale.x = 1.0;
  pathVehicle.scale.y = 1.0;
  pathVehicle.scale.z = 1.0;

  double node_x = node.getX() * Constants::cellSize;
  double node_y = node.getY() * Constants::cellSize;
  double node_yaw = node.getT();

  const float triangle_length = 1.0f * scale;
  const float triangle_width = 0.6f * scale;

  std::vector<Eigen::Vector2d> local_triangle;
  local_triangle.push_back(Eigen::Vector2d(triangle_length, 0.0f));
  local_triangle.push_back(Eigen::Vector2d(-triangle_length * 0.3f, triangle_width));
  local_triangle.push_back(Eigen::Vector2d(-triangle_length * 0.3f, -triangle_width));

  Eigen::Rotation2Dd rotation(node_yaw);
  Eigen::Vector2d translation(node_x, node_y);

  std::vector<Eigen::Vector2d> global_triangle;
  for (const auto& p_local : local_triangle) {
    global_triangle.push_back(rotation * p_local + translation);
  }

  geometry_msgs::Point p0, p1, p2;
  p0.x = global_triangle[0].x(); p0.y = global_triangle[0].y(); p0.z = 0.5;
  p1.x = global_triangle[1].x(); p1.y = global_triangle[1].y(); p1.z = 0.5;
  p2.x = global_triangle[2].x(); p2.y = global_triangle[2].y(); p2.z = 0.5;

  pathVehicle.points.push_back(p0);
  pathVehicle.points.push_back(p1);
  pathVehicle.points.push_back(p2);

  if (clear) {
    visualization_msgs::Marker clear_marker;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    clear_marker.ns = pathVehicle.ns;
    clear_marker.header.frame_id = "map";
    pathVehicles.markers.push_back(clear_marker);
  }

  pathVehicles.markers.push_back(pathVehicle);
  pubPathVehicles.publish(pathVehicles);
}

void Path::add_occupy( std::vector<Eigen::Vector2d> obs,int num_tip) {

  visualization_msgs::Marker Vehicle_occupy;

  Vehicle_occupy.action = visualization_msgs::Marker::ADD;
  Vehicle_occupy.header.frame_id = "map";
  Vehicle_occupy.header.stamp = ros::Time(0);
  Vehicle_occupy.id = num_tip;
  Vehicle_occupy.ns = "car_occupied";

  Vehicle_occupy.type = visualization_msgs::Marker::POINTS;

  Vehicle_occupy.scale.x = 0.2;
  Vehicle_occupy.scale.y = 0.2;
  Vehicle_occupy.scale.z = 0.0;

  Vehicle_occupy.color.r = 1.0;
  Vehicle_occupy.color.g = 0.0;
  Vehicle_occupy.color.b = 0.0;
  Vehicle_occupy.color.a = 1.0;

  Vehicle_occupy.points.clear();
  for(size_t j = 0; j < obs.size(); j++){
    geometry_msgs::Point point;
    point.x = obs[j].x();
    point.y = obs[j].y();
    point.z = 0.5;
    Vehicle_occupy.points.push_back(point);
  }

  pathNodes.markers.push_back(Vehicle_occupy);

  pubPathNodes.publish(pathNodes);

  ros::Duration(0.5).sleep();
}

void Path::addVehicleOutlineEdges(const Node3D& node, int i, PathColor color) {

  const std::vector<Eigen::Vector2d>& local_shape = HybridAStar::Constants::my_car_shape_for_show();

  if (local_shape.size() < 2) {
    ROS_WARN("Vehicle shape has less than 2 points, cannot draw edges.");
    return;
  }

  double node_x = node.getX() * Constants::cellSize;
  double node_y = node.getY() * Constants::cellSize;
  double node_yaw = node.getT();

  Eigen::Rotation2Dd rotation(node_yaw);
  Eigen::Vector2d translation(node_x, node_y);

  visualization_msgs::Marker edgesMarker;

  if (i == 1) {
    edgesMarker.action = visualization_msgs::Marker::DELETEALL;
  } else {
    edgesMarker.action = visualization_msgs::Marker::ADD;
  }

  edgesMarker.header.frame_id = "map";
  edgesMarker.header.stamp = ros::Time::now();
  edgesMarker.id = i;
  edgesMarker.ns = "vehicle_outline_edges";
  edgesMarker.type = visualization_msgs::Marker::LINE_LIST;
  edgesMarker.color.a = 0.5;
  edgesMarker.color.r = color.r;
  edgesMarker.color.g = color.g;
  edgesMarker.color.b = color.b;
  edgesMarker.scale.x = 0.05;
  edgesMarker.scale.y = 0.05;

  for (size_t j = 0; j < local_shape.size(); ++j) {
    geometry_msgs::Point start_point, end_point;

    Eigen::Vector2d current_global = rotation * local_shape[j] + translation;
    start_point.x = current_global.x();
    start_point.y = current_global.y();
    start_point.z = 0.5;

    size_t next_j = (j + 1) % local_shape.size();
    Eigen::Vector2d next_global = rotation * local_shape[next_j] + translation;
    end_point.x = next_global.x();
    end_point.y = next_global.y();
    end_point.z = 0.5;

    edgesMarker.points.push_back(start_point);
    edgesMarker.points.push_back(end_point);
  }

  pathNodes.markers.push_back(edgesMarker);
  pubPathNodes.publish(pathNodes);
}
