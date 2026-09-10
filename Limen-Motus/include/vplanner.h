#ifndef VPLANNER_H
#define VPLANNER_H

#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>

#include "dynamicvoronoi.h"
#include "node3d.h"
#include "vector2d.h"
#include "helper.h"
#include "constants.h"
namespace HybridAStar {

namespace VelocityConstants {
    const double MAX_VELOCITY = 2.0;
    const double MAX_ACCELERATION = 0.5;
    const double MAX_DECELERATION = 3.0;
    const double MAX_CURVATURE_VELOCITY = 0.5;
    const double SAFETY_TIME_BUFFER = 1.0;
    const double MIN_VELOCITY = 0.0001;
}

struct VelocityPoint {
    float x, y, t;
    double velocity;
    double acceleration;
    double time_from_start;
    double time_after_before;
    double distance_from_start;

    VelocityPoint(float x = 0, float y = 0, float t = 0, double v = 0, double a = 0)
        : x(x), y(y), t(t), velocity(v), acceleration(a), time_from_start(0), time_after_before(0), distance_from_start(0) {}
};

class VelocityPlanner {
public:
    VelocityPlanner() {}

    std::vector<VelocityPoint> planVelocity(const std::vector<Node3D>& path,
                                           double target_velocity = VelocityConstants::MAX_VELOCITY) ;

private:

    std::vector<double> calculateDistances(const std::vector<Node3D>& path) ;

    std::vector<double> calculateCurvatures(const std::vector<Node3D>& path) ;

    std::vector<double> calculateMaxVelocities(const std::vector<double>& curvatures, double target_velocity) ;

    std::vector<double> forwardPass(const std::vector<double>& distances,
                                  const std::vector<double>& max_velocities,
                                  double initial_velocity) ;

    std::vector<double> backwardPass(const std::vector<double>& distances,
                                   const std::vector<double>& forward_velocities,
                                   const std::vector<double>& max_velocities) ;

    std::vector<double> calculateAccelerations(const std::vector<double>& velocities,
                                             const std::vector<double>& distances) ;

    std::vector<double> calculateTimes(const std::vector<double>& velocities,
                                     const std::vector<double>& distances) ;
};

void printVelocityProfile(const std::vector<VelocityPoint>& profile);

}
#endif
