#ifndef PATH_H
#define PATH_H

#include <iostream>
#include <cstring>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "node3d.h"
#include "constants.h"
#include "helper.h"
#include "vplanner.h"
namespace HybridAStar {

struct PathColor {
    float r, g, b, a;
    PathColor(float r = 0.0, float g = 0.0, float b = 0.0, float a = 0.1)
        : r(r), g(g), b(b), a(a) {}
};

class Path {
 public:

  Path(bool smoothed = false) {
    std::string pathTopic = "/path";
    std::string pathNodesTopic = "/pathNodes";
    std::string pathVehicleTopic = "/pathVehicle";
    std::string pathVehicleTopic_forever = "/pathVehicle_forever";

    if (smoothed) {
      pathTopic = "/sPath";
      pathNodesTopic = "/sPathNodes";
      pathVehicleTopic = "/sPathVehicle";
      this->smoothed = smoothed;
    }

    pubPath = n.advertise<nav_msgs::Path>(pathTopic, 1);
    pubPathNodes = n.advertise<visualization_msgs::MarkerArray>(pathNodesTopic, 1);
    pubPathVehicles = n.advertise<visualization_msgs::MarkerArray>(pathVehicleTopic, 1);
    pubPathVehicles_forever = n.advertise<visualization_msgs::MarkerArray>(pathVehicleTopic_forever, 1);

    path.header.frame_id = "path";

   precomputedCarShapeTriangles= earClipping(Constants::my_car_shape_for_show());
   trianglesInitialized=true;
  }

  void updatePath(const std::vector<Node3D> &nodePath);

  void updatePath_onlynode(const std::vector<Node3D> &nodePath);

   int updatePathWithColor(const std::vector<Node3D>& nodePath, PathColor color,int k);

   void updatePath_v(const std::vector<Node3D> &nodePath,
   const std::vector<VelocityPoint> velocity_profile);

  void addSegment(const Node3D& node, std::string visaul_choose = "onebyone");

  void addNode(const Node3D& node, int i, std::string visaul_choose = "onebyone");
  void addNode_bigger(const Node3D& node, int i, std::string visaul_choose = "onebyone");

  void addVehicle(const Node3D& node, int i, std::string visaul_choose = "onebyone");
  void addVehicle_SPMT(const Node3D& node, int i, bool clear);
  void addVehicle_SPMT_earcut( const Node3D& node, int i, bool clear_body,bool clear_outline, PathColor color= PathColor(1.0, 0.0, 0.0, 0.7));
  void addVehicle_SPMT_earcut_forever(const Node3D& node, int i, bool clear, PathColor color = PathColor(1.0, 0.0, 0.0, 0.7));

  void addVehicle_TriangleMarker(const Node3D& node, int i, bool clear, PathColor color = PathColor(1.0, 0.0, 0.0, 0.7), float scale = 1.0f);

  void clear();
  void clear_vehicle();

  void publishPath() { pubPath.publish(path); }

  void publishPathNodes() { pubPathNodes.publish(pathNodes); }

  void publishPathVehicles() { pubPathVehicles.publish(pathVehicles); }
  void add_occupy( std::vector<Eigen::Vector2d> obs ,int num_tip) ;
  void addVehicleOutlineEdges(const Node3D& node, int i, PathColor color = PathColor(1.0, 0.0, 0.0, 0.5));

   inline double crossProduct(const Eigen::Vector2d& p0, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
      return (p1.x() - p0.x()) * (p2.y() - p0.y()) - (p1.y() - p0.y()) * (p2.x() - p0.x());
   }

   inline bool pointInTriangle(const Eigen::Vector2d& p, const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& c) {
      double d1 = crossProduct(p, a, b);
      double d2 = crossProduct(p, b, c);
      double d3 = crossProduct(p, c, a);

      bool has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
      bool has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

      return !(has_neg && has_pos);
   }

   inline std::vector<std::vector<int>> earClipping(const std::vector<Eigen::Vector2d>& polygon) {
      int n = polygon.size();
      if (n < 3) return {};

      std::vector<int> V(n);
      for (int i = 0; i < n; ++i) V[i] = i;

      std::vector<std::vector<int>> triangles;

      while (V.size() > 3) {
         bool ear_found = false;
         for (size_t i = 0; i < V.size(); ++i) {
               int prev_idx = V[(i == 0) ? V.size() - 1 : i - 1];
               int curr_idx = V[i];
               int next_idx = V[(i == V.size() - 1) ? 0 : i + 1];

               if (crossProduct(polygon[prev_idx], polygon[curr_idx], polygon[next_idx]) <= 0) {
                  continue;
               }

               bool is_ear = true;

               for (size_t j = 0; j < V.size(); ++j) {
                  if (V[j] == prev_idx || V[j] == curr_idx || V[j] == next_idx) continue;
                  if (pointInTriangle(polygon[V[j]], polygon[prev_idx], polygon[curr_idx], polygon[next_idx])) {
                     is_ear = false;
                     break;
                  }
               }

               if (is_ear) {
                  triangles.push_back({prev_idx, curr_idx, next_idx});
                  V.erase(V.begin() + i);
                  ear_found = true;
                  break;
               }
         }

         if (!ear_found) {

               ROS_ERROR("Ear Clipping Failed: Could not find an ear. The polygon might be invalid.");
               return {};
         }
      }

      if (V.size() == 3) {
         triangles.push_back({V[0], V[1], V[2]});
      }
      return triangles;
   }

   inline static std::vector<std::vector<int>>get_trigle(){
      return  precomputedCarShapeTriangles;
   }
 private:

  ros::NodeHandle n;

  ros::Publisher pubPath;

  ros::Publisher pubPathNodes;

  ros::Publisher pubPathVehicles;
  ros::Publisher pubPathVehicles_forever;

  nav_msgs::Path path;

  visualization_msgs::MarkerArray pathNodes;

  visualization_msgs::MarkerArray pathVehicles;
  visualization_msgs::MarkerArray pathVehicles_forever;

  bool smoothed = false;

  static std::vector<std::vector<int>> precomputedCarShapeTriangles;
  static bool trianglesInitialized;
};
}
#endif
