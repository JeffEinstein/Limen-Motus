#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include <filesystem>
#include <fstream>

#include <nav_msgs/OccupancyGrid.h>
#include <nlohmann/json.hpp>

#include "constants.h"
#include "lookup.h"
#include "node2d.h"
#include "node3d.h"

namespace HybridAStar {
namespace {
inline void getConfiguration(const Node2D* node, float& x, float& y, float& t) {
  x = node->getX();
  y = node->getY();

  t = 99;
}

inline void getConfiguration(const Node3D* node, float& x, float& y, float& t) {
  x = node->getX();
  y = node->getY();
  t = node->getT();
}
}

class CollisionDetection {
 public:

  CollisionDetection();

  template<typename T> bool isTraversable(const T* node) const {

    float cost = 0;
    float x;
    float y;
    float t;

    getConfiguration(node, x, y, t);

    if (t == 99) {
      return !grid->data[node->getIdx()];
    }

    if (true) {

      cost = configurationTest_polygon(x, y, t) ? 0 : 1;
    } else {

      cost = configurationTest_polygon(x, y, t);

    }

    return cost <= 0;
  }

  float configurationCost(float x, float y, float t) const {return 0;}

  bool configurationTest(float x, float y, float t) const;

  bool configurationTest_polygon(float x, float y, float t) const;

  void updateGrid(nav_msgs::OccupancyGrid::Ptr map) {grid = map;}
  void set_triagle(std::vector<std::vector<int>> triagle) {body_triagle = triagle;}

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

   inline std::vector<std::vector<int>> earClipping(const std::vector<Eigen::Vector2d>& polygon,
                                                    bool save_to_json = false) {
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

      if (save_to_json) {
         const std::filesystem::path output_path(
             "/workspaces/ros-noetic-ws/src/paintforpaper1/obsdetection/triangles.json");
         nlohmann::json j;
         j["body_triagle"] = triangles;

         try {
            std::filesystem::create_directories(output_path.parent_path());
            std::ofstream output_file(output_path);
            if (output_file.is_open()) {
               output_file << j.dump(4) << std::endl;
            } else {
               ROS_ERROR_STREAM("Failed to open triangle output file: " << output_path);
            }
         } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Failed to save ear clipping result: " << e.what());
         }
      }

      return triangles;
   }

   std::vector<std::vector<int>>get_trigle(){
      return  body_triagle;
   }

    std::vector<Eigen::Vector2d>  returnpolgen(float x, float y, float t, bool savetofile = false) const;
    nav_msgs::OccupancyGrid::Ptr get_grid(){return grid;}

 private:

  nav_msgs::OccupancyGrid::Ptr grid;

  Constants::config collisionLookup[Constants::headings * Constants::positions];
  Constants::config collisionLookup_polygon[Constants::headings * Constants::positions];

  std::vector<std::vector<int>> body_triagle;
  bool trianglesInitialized =false;
};
}
#endif
