#ifndef SHOW_H
#define SHOW_H
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "node3d.h"
namespace HybridAStar {

class Path_plot {
 public:
    Path_plot() ;

   void publishPath(Node3D* pathnode) ;
 private:

  ros::Publisher path_pub_;
};
}
#endif
