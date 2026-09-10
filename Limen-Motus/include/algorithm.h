#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>

typedef ompl::base::SE2StateSpace::StateType State;

#include "node3d.h"
#include "node2d.h"
#include "visualize.h"
#include "collisiondetection.h"
#include "node_aws.h"

#include <vector>
#include <cmath>
#include <iostream>
#include <unordered_set>
#include <algorithm>
#include <boost/heap/binomial_heap.hpp>

#include <Eigen/Dense>

#include "show.h"
namespace HybridAStar {
class Node3D;
class Node2D;
class Visualize;

class Algorithm {
 public:

  Algorithm() {}

  static Node3D* hybridAStar(Node3D& start,
                             const Node3D& goal,
                             Node3D* nodes3D,
                             Node2D* nodes2D,
                             int width,
                             int height,
                             CollisionDetection& configurationSpace,
                             float* dubinsLookup,
                             Visualize& visualization);

   static Node3D* hybridAStar_M(Node3D& start,
                               const Node3D& goal,
                               Node3D* nodes3D,
                               int width,
                               int height,
                               CollisionDetection& configurationSpace,
                               float* dubinsLookup,

                               std::vector<int>* dsuParent = nullptr,
                               std::vector<int>* dsuRank = nullptr,
                               bool* dsuInitialized = nullptr,

                               double* t_judge_output_Astar = nullptr,
                               double* t_judge_output_DSU = nullptr,
                               double* t_planning_output = nullptr,

                               bool *path_is_feasible_Astar = nullptr,
                               bool *path_is_feasible_DSU = nullptr,

                               bool *path_is_feasible_searching = nullptr,
                               int *planning_iterations_output = nullptr,
                               double *path_cost_output = nullptr
                               );

   static NodeAWS* hybridAStar_aws(NodeAWS& start,
                                 const NodeAWS& goal,
                                 NodeAWS* nodes3D,
                                 Node2D* nodes2D,
                                 int width,
                                 int height,
                                 CollisionDetection& configurationSpace,
                                 float* dubinsLookup,
                                 Visualize& visualization,
                                 bool showflag);
   static void releaseAwsSearchMemory();
   };
}
#endif
