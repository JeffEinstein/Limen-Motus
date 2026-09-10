#include "vplanner.h"
#include <iomanip>
namespace HybridAStar {

void printVelocityProfile(const std::vector<VelocityPoint>& profile) {
    std::cout << "Velocity Profile:" << std::endl;
    std::cout << "Index\tX\tY\tTheta\tVel(m/s)\tAcc(m/s²)\tTime(s)\tDist(m)" << std::endl;

    for (size_t i = 0; i < profile.size(); ++i) {
        const auto& vp = profile[i];
        std::cout << i << "\t"
                  << std::fixed << std::setprecision(2)
                  << vp.x << "\t" << vp.y << "\t" << vp.t << "\t"
                  << vp.velocity << "\t\t" << vp.acceleration << "\t\t"
                  << vp.time_from_start << "\t" << vp.distance_from_start << std::endl;
    }
}

std::vector<VelocityPoint> VelocityPlanner::planVelocity(const std::vector<Node3D>& path,
                                        double target_velocity) {
    if (path.size() < 2) {
        std::cerr << "Path too short for velocity planning" << std::endl;
        return {};
    }

    std::vector<VelocityPoint> velocity_profile;
    velocity_profile.reserve(path.size());

    std::vector<double> distances = calculateDistances(path);
    std::vector<double> curvatures = calculateCurvatures(path);

    std::vector<double> max_velocities = calculateMaxVelocities(curvatures, target_velocity);

    std::vector<double> forward_velocities = forwardPass(distances, max_velocities, VelocityConstants::MIN_VELOCITY);

    std::vector<double> final_velocities = backwardPass(distances, forward_velocities, max_velocities);

    std::vector<double> accelerations = calculateAccelerations(final_velocities, distances);
    std::vector<double> times = calculateTimes(final_velocities, distances);

    double cumulative_time = 0.0;
    double cumulative_distance = 0.0;

    double last_time=0.0;
    for (size_t i = 0; i < path.size(); ++i) {

        VelocityPoint vp(path[i].getX(), path[i].getY(), path[i].getT(),
                        final_velocities[i], accelerations[i]);
        vp.time_from_start = cumulative_time;
        vp.distance_from_start = cumulative_distance;
        vp.time_after_before= times[i];
        velocity_profile.push_back(vp);

        if (i < path.size() - 1) {
            cumulative_time += times[i];
            cumulative_distance += distances[i];
        }

    }

    return velocity_profile;
}

    std::vector<double> VelocityPlanner::calculateDistances(const std::vector<Node3D>& path) {
        std::vector<double> distances;
        distances.reserve(path.size() - 1);

        for (size_t i = 0; i < path.size() - 1; ++i) {
            double dx = path[i+1].getX() - path[i].getX();
            double dy = path[i+1].getY() - path[i].getY();
            double distance = sqrt(dx * dx + dy * dy);
            distances.push_back(std::max(distance, 0.01));
        }

        return distances;
    }

    std::vector<double> VelocityPlanner::calculateCurvatures(const std::vector<Node3D>& path) {
        std::vector<double> curvatures(path.size(), 0.0);

        for (size_t i = 1; i < path.size() - 1; ++i) {

            double x1 = path[i-1].getX(), y1 = path[i-1].getY();
            double x2 = path[i].getX(), y2 = path[i].getY();
            double x3 = path[i+1].getX(), y3 = path[i+1].getY();

            double dx1 = x2 - x1, dy1 = y2 - y1;
            double dx2 = x3 - x2, dy2 = y3 - y2;

            double len1 = sqrt(dx1 * dx1 + dy1 * dy1);
            double len2 = sqrt(dx2 * dx2 + dy2 * dy2);

            if (len1 > 0.001 && len2 > 0.001) {

                dx1 /= len1; dy1 /= len1;
                dx2 /= len2; dy2 /= len2;

                double cross_product = dx1 * dy2 - dy1 * dx2;
                double dot_product = dx1 * dx2 + dy1 * dy2;
                double theta_change = atan2(cross_product, dot_product);

                double arc_length = (len1 + len2) / 2.0;
                curvatures[i] = std::abs(theta_change) / arc_length;
            }
        }

        if (path.size() > 2) {
            curvatures[0] = curvatures[1];
            curvatures[path.size()-1] = curvatures[path.size()-2];
        }

        return curvatures;
    }

    std::vector<double> VelocityPlanner::calculateMaxVelocities(const std::vector<double>& curvatures, double target_velocity) {
        std::vector<double> max_velocities;
        max_velocities.reserve(curvatures.size());

        for (double curvature : curvatures) {
            double curvature_velocity = VelocityConstants::MAX_CURVATURE_VELOCITY;
            if (curvature > 0.001) {

                curvature_velocity = sqrt(VelocityConstants::MAX_ACCELERATION / curvature);
            }

            double max_vel = std::min({target_velocity, curvature_velocity, VelocityConstants::MAX_VELOCITY});
            max_velocities.push_back(std::max(max_vel, VelocityConstants::MIN_VELOCITY));
        }

        return max_velocities;
    }

    std::vector<double> VelocityPlanner::forwardPass(const std::vector<double>& distances,
                                  const std::vector<double>& max_velocities,
                                  double initial_velocity) {
        std::vector<double> velocities = max_velocities;
        velocities[0] = std::min(initial_velocity, max_velocities[0]);

        for (size_t i = 0; i < distances.size(); ++i) {
            double current_vel = velocities[i];
            double distance = distances[i];

            double max_next_vel_squared = current_vel * current_vel +
                                        2 * VelocityConstants::MAX_ACCELERATION * distance;
            double max_next_vel = sqrt(max_next_vel_squared);

            velocities[i+1] = std::min(velocities[i+1], max_next_vel);
        }

        return velocities;
    }

    std::vector<double> VelocityPlanner::backwardPass(const std::vector<double>& distances,
                                   const std::vector<double>& forward_velocities,
                                   const std::vector<double>& max_velocities) {
        std::vector<double> velocities = forward_velocities;

        for (int i = distances.size() - 1; i >= 0; --i) {
            double next_vel = velocities[i+1];
            double distance = distances[i];

            double max_current_vel_squared = next_vel * next_vel +
                                           2 * VelocityConstants::MAX_DECELERATION * distance;
            double max_current_vel = sqrt(max_current_vel_squared);

            velocities[i] = std::min(velocities[i], max_current_vel);
            velocities[i] = std::max(velocities[i], VelocityConstants::MIN_VELOCITY);
        }

        return velocities;
    }

    std::vector<double> VelocityPlanner::calculateAccelerations(const std::vector<double>& velocities,
                                             const std::vector<double>& distances) {
        std::vector<double> accelerations(velocities.size(), 0.0);

        for (size_t i = 0; i < distances.size(); ++i) {
            double v1 = velocities[i];
            double v2 = velocities[i+1];
            double distance = distances[i];

            if (distance > 0.001) {
                accelerations[i] = (v2 * v2 - v1 * v1) / (2.0 * distance);
            }
        }

        return accelerations;
    }

    std::vector<double> VelocityPlanner::calculateTimes(const std::vector<double>& velocities,
                                     const std::vector<double>& distances) {
        std::vector<double> times;
        times.reserve(distances.size());

        for (size_t i = 0; i < distances.size(); ++i) {
            double v1 = velocities[i];
            double v2 = velocities[i+1];
            double distance = distances[i];

            double avg_velocity = (v1 + v2) / 2.0;
            if (avg_velocity > 0.001) {
                times.push_back(distance / avg_velocity);
            } else {
                times.push_back(0.0);
            }
        }

        return times;
    }
};
