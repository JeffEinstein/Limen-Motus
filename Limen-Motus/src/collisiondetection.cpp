#include "collisiondetection.h"

#include <filesystem>
#include <fstream>

#include <nlohmann/json.hpp>

#include"matplotlibcpp.h"
namespace plt =matplotlibcpp;
#include <Python.h>
using namespace HybridAStar;
using json = nlohmann::json;

CollisionDetection::CollisionDetection() {
  this->grid = nullptr;

   body_triagle= earClipping(Constants::my_car_shape_for_collision(), true);
   trianglesInitialized=true;

  if(HybridAStar::Constants::Convex_Method=="triangle"){
   Lookup::collisionLookup_polygon(collisionLookup_polygon, false, body_triagle,true);
  }else if(HybridAStar::Constants::Convex_Method=="Ellipse"){
    Lookup::collisionLookup_polygon_Ellipse(collisionLookup_polygon, false);
  }else{
   Lookup::collisionLookup_polygon(collisionLookup_polygon, false, body_triagle);
  }

}

bool CollisionDetection::configurationTest(float x, float y, float t) const {
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

  int X = (int)x;
  int Y = (int)y;
  int iX = (int)((x - (long)x) * positionResolution);
  iX = iX > 0 ? iX : 0;
  int iY = (int)((y - (long)y) * positionResolution);
  iY = iY > 0 ? iY : 0;
  int iT = (int)(t / deltaHeadingRad);
  int idx = iY * positionResolution * headings + iX * headings + iT;
  int cX;
  int cY;

  for (int i = 0; i < collisionLookup[idx].length; ++i) {
    cX = (X + collisionLookup[idx].pos[i].x);
    cY = (Y + collisionLookup[idx].pos[i].y);

    if (cX >= 0 && (unsigned int)cX < grid->info.width && cY >= 0 && (unsigned int)cY < grid->info.height) {
      if (grid->data[cY * grid->info.width + cX]) {
        return false;
      }
    }else{return false;}
  }

  return true;
}

bool CollisionDetection::configurationTest_polygon(float x, float y, float t) const {

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

  int X = (int)x;
  int Y = (int)y;
  int iX = (int)((x - (long)x) * positionResolution);
  iX = iX > 0 ? iX : 0;
  int iY = (int)((y - (long)y) * positionResolution);
  iY = iY > 0 ? iY : 0;
  int iT = (int)(t / deltaHeadingRad);
  int idx = iY * positionResolution * headings + iX * headings + iT;

  int max_idx = positionResolution * positionResolution * headings;
  if (idx < 0 || idx >= max_idx) {

    return false;
  }

  int cX;
  int cY;

  int lenth = collisionLookup_polygon[idx].length;

  for (int i = 0; i < collisionLookup_polygon[idx].length; ++i) {
    cX = (X + collisionLookup_polygon[idx].pos[i].x);
    cY = (Y + collisionLookup_polygon[idx].pos[i].y);

    if (cX >= 0 && (unsigned int)cX < grid->info.width && cY >= 0 && (unsigned int)cY < grid->info.height) {
      int m=cY * grid->info.width + cX;

      if (grid->data[m]) {
        return false;
      }
    }else{
      return false;
    }

  }

  return true;
}

std::vector<Eigen::Vector2d> CollisionDetection::returnpolgen(float x, float y, float t, bool savetofile) const {
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
  int X = (int)x;
  int Y = (int)y;
  int iX = (int)((x - (long)x) * positionResolution);
  iX = iX > 0 ? iX : 0;
  int iY = (int)((y - (long)y) * positionResolution);
  iY = iY > 0 ? iY : 0;
  int iT = (int)(t / deltaHeadingRad);
  int idx = iY * positionResolution * headings + iX * headings + iT;
  const int max_idx = positionResolution * positionResolution * headings;
  if (idx < 0 || idx >= max_idx) {
    return {};
  }

  int cX;
  int cY;
  std::vector<Eigen::Vector2d> body_obs;
  for (int i = 0; i < collisionLookup_polygon[idx].length; ++i) {
    cX = (X + collisionLookup_polygon[idx].pos[i].x);
    cY = (Y + collisionLookup_polygon[idx].pos[i].y);
    body_obs.push_back(Eigen::Vector2d(cX, cY));

  }

  if (savetofile) {
    const double sampled_x = static_cast<double>(X) + static_cast<double>(iX) / positionResolution;
    const double sampled_y = static_cast<double>(Y) + static_cast<double>(iY) / positionResolution;
    const double sampled_t = static_cast<double>(iT) * deltaHeadingRad;
    const std::filesystem::path output_path(
        "/workspaces/ros-noetic-ws/src/paintforpaper1/obsdetection/obca_withedge.json");

    json j;
    j["x"] = sampled_x;
    j["y"] = sampled_y;
    j["t"] = sampled_t;
    j["body_obs"] = json::array();

    for (const auto& point : body_obs) {
      j["body_obs"].push_back({
          {"x", point.x()},
          {"y", point.y()}
      });
    }

    try {
      std::filesystem::create_directories(output_path.parent_path());
      std::ofstream output_file(output_path);
      if (output_file.is_open()) {
        output_file << j.dump(4) << std::endl;
      } else {
        std::cerr << "Failed to open file for writing: " << output_path << std::endl;
      }
    } catch (const std::exception& e) {
      std::cerr << "Failed to save obstacle data to JSON: " << e.what() << std::endl;
    }
  }

  return body_obs;
}
