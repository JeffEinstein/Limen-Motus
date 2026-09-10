#include "planner.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <chrono>
#include <cstdlib>

#define show_car_occupied

#define ANT

using json = nlohmann::json;

using namespace HybridAStar;

namespace {
const std::vector<int>& selectedParkingIndices() {
    static const std::vector<int> indices = []() {
        std::vector<int> parsed;
        const char* value = std::getenv("HYBRID_ASTAR_PARKING_INDICES");
        if (value == nullptr || *value == '\0') {
            return parsed;
        }

        std::stringstream stream(value);
        std::string token;
        const int parking_count = static_cast<int>(HybridAStar::Constants::parking_plot().size());
        while (std::getline(stream, token, ',')) {
            if (token.empty()) {
                continue;
            }
            const int one_based = std::stoi(token);
            if (one_based < 1 || one_based > parking_count) {
                throw std::out_of_range("parking index must be between 1 and " + std::to_string(parking_count));
            }
            const int zero_based = one_based - 1;
            if (std::find(parsed.begin(), parsed.end(), zero_based) != parsed.end()) {
                throw std::invalid_argument("duplicate parking index: " + std::to_string(one_based));
            }
            parsed.push_back(zero_based);
        }

        std::cout << "Selected parking indices:";
        for (const int index : parsed) {
            std::cout << ' ' << index + 1;
        }
        std::cout << std::endl;
        return parsed;
    }();
    return indices;
}

const std::vector<int>& selectedOnsiteIndices() {
    static const std::vector<int> indices = []() {
        std::vector<int> parsed;
        const char* value = std::getenv("HYBRID_ASTAR_ONSITE_INDICES");
        if (value == nullptr || *value == '\0') {
            return parsed;
        }

        std::stringstream stream(value);
        std::string token;
        while (std::getline(stream, token, ',')) {
            if (token.empty()) {
                continue;
            }
            const int one_based = std::stoi(token);
            if (one_based < 1 || one_based > 450) {
                throw std::out_of_range("onsite index must be between 1 and 450");
            }
            const int zero_based = one_based - 1;
            if (std::find(parsed.begin(), parsed.end(), zero_based) != parsed.end()) {
                throw std::invalid_argument("duplicate onsite index: " + std::to_string(one_based));
            }
            parsed.push_back(zero_based);
        }
        return parsed;
    }();
    return indices;
}

int loadOnsitePairsFromJson(HybridAStar::Constants::Pathpair* pair, const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open onsite pair file: " << filename << std::endl;
        return 0;
    }

    json data;
    try {
        file >> data;
    } catch (const std::exception& e) {
        std::cerr << "Failed to parse onsite pair file: " << e.what() << std::endl;
        return 0;
    }

    if (!data.is_array()) {
        std::cerr << "Onsite pair file should contain a JSON array: " << filename << std::endl;
        return 0;
    }

    int count = 0;
    for (const auto& item : data) {
        if (count >= 560) {
            break;
        }
        if (!item.contains("start") || !item.contains("goal")) {
            continue;
        }
        const auto& s = item["start"];
        const auto& g = item["goal"];
        pair[count] = HybridAStar::Constants::Pathpair();
        pair[count].start_x = s.value("x", 0.0);
        pair[count].start_y = s.value("y", 0.0);
        pair[count].start_yaw = s.value("yaw", 0.0);
        pair[count].goal_x = g.value("x", 0.0);
        pair[count].goal_y = g.value("y", 0.0);
        pair[count].goal_yaw = g.value("yaw", 0.0);
        if (item.contains("test_info") && item["test_info"].is_object()) {
            pair[count].radi_test = item["test_info"].value("radius", 0);
        }
        ++count;
    }

    std::cout << "Loaded onsite fixed pairs: " << count << " from " << filename << std::endl;
    return count;
}

int loadExistingResultsFromJson(HybridAStar::Constants::Pathpair* pair,
                                const std::string& filename,
                                int max_count) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open resume result file: " << filename << std::endl;
        return 0;
    }

    json data;
    try {
        file >> data;
    } catch (const std::exception& e) {
        std::cerr << "Failed to parse resume result file: " << e.what() << std::endl;
        return 0;
    }

    if (!data.is_array()) {
        std::cerr << "Resume result file should contain a JSON array: " << filename << std::endl;
        return 0;
    }

    const int count = std::min({max_count, static_cast<int>(data.size()), 560});
    for (int i = 0; i < count; ++i) {
        const auto& item = data[i];
        if (!item.contains("start") || !item.contains("goal")) {
            std::cerr << "Resume result " << i << " has no start or goal." << std::endl;
            return i;
        }

        auto& result = pair[i];
        const auto& s = item["start"];
        const auto& g = item["goal"];
        result.start_x = s.value("x", 0.0);
        result.start_y = s.value("y", 0.0);
        result.start_yaw = s.value("yaw", 0.0);
        result.goal_x = g.value("x", 0.0);
        result.goal_y = g.value("y", 0.0);
        result.goal_yaw = g.value("yaw", 0.0);
        result.path_length = item.value("path_length", 0.0);
        result.path_cost = item.value("path_cost", 0.0);
        result.length_t = item.value("length_t", 0);

        if (item.contains("time") && item["time"].is_object()) {
            const auto& timing = item["time"];
            result.t_judge_output_Astar = timing.value("judge_astar", 0.0);
            result.t_judge_output_DSU = timing.value("judge_dsu", 0.0);
            result.t_planning_output = timing.value("planning", 0.0);
            result.planning_iterations = timing.value("planning_iterations", 0);
        }
        if (item.contains("feasible") && item["feasible"].is_object()) {
            const auto& feasible = item["feasible"];
            result.path_is_feasible_Astar = feasible.value("astar", false);
            result.path_is_feasible_DSU = feasible.value("dsu", false);
            result.path_is_feasible_searching = feasible.value("searching", false);
        }
        if (item.contains("test_info") && item["test_info"].is_object()) {
            result.radi_test = item["test_info"].value("radius", 0);
        }

        result.path_xyt.clear();
        if (item.contains("path_points") && item["path_points"].is_array()) {
            for (const auto& point : item["path_points"]) {
                result.path_xyt.emplace_back(
                    point.value("x", 0.0),
                    point.value("y", 0.0),
                    point.value("theta", 0.0));
            }
        }
        result.result_valid = true;
    }

    std::cout << "Resumed " << count << " existing results from " << filename << std::endl;
    return count;
}

bool runOnsiteEndpointCollisionCheck(const HybridAStar::Constants::Pathpair* pairs,
                                     int pair_count,
                                     const CollisionDetection& configuration_space) {
    const char* output_value = std::getenv("HYBRID_ASTAR_ENDPOINT_COLLISION_OUTPUT");
    if (output_value == nullptr || *output_value == '\0') {
        return false;
    }

    json report;
    report["metadata"] = {
        {"collision_method", HybridAStar::Constants::Convex_Method},
        {"collision_shape", "cargo_shape_ellipse"},
        {"pair_source", HybridAStar::Constants::Experiment_Onsite_450_Pairs_File}
    };
    report["collision_shape_points"] = json::array();
    for (const auto& point : HybridAStar::Constants::my_car_shape_for_collision()) {
        report["collision_shape_points"].push_back({point.x(), point.y()});
    }

    int start_collision_count = 0;
    int goal_collision_count = 0;
    int either_collision_count = 0;
    int both_collision_count = 0;
    report["tasks"] = json::array();

    const auto begin = std::chrono::steady_clock::now();
    for (int i = 0; i < pair_count; ++i) {
        const auto& pair = pairs[i];
        const float start_yaw = Helper::normalizeHeadingRad(pair.start_yaw);
        const float goal_yaw = Helper::normalizeHeadingRad(pair.goal_yaw);
        const bool start_traversable = configuration_space.configurationTest_polygon(
            pair.start_x, pair.start_y, start_yaw);
        const bool goal_traversable = configuration_space.configurationTest_polygon(
            pair.goal_x, pair.goal_y, goal_yaw);
        const bool start_collision = !start_traversable;
        const bool goal_collision = !goal_traversable;

        start_collision_count += start_collision ? 1 : 0;
        goal_collision_count += goal_collision ? 1 : 0;
        either_collision_count += (start_collision || goal_collision) ? 1 : 0;
        both_collision_count += (start_collision && goal_collision) ? 1 : 0;

        report["tasks"].push_back({
            {"task_index", i + 1},
            {"start", {{"x", pair.start_x}, {"y", pair.start_y}, {"yaw", pair.start_yaw}}},
            {"goal", {{"x", pair.goal_x}, {"y", pair.goal_y}, {"yaw", pair.goal_yaw}}},
            {"start_collision", start_collision},
            {"goal_collision", goal_collision},
            {"endpoint_feasible", !start_collision && !goal_collision}
        });
    }
    const auto elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - begin).count();

    report["summary"] = {
        {"task_count", pair_count},
        {"endpoint_feasible_count", pair_count - either_collision_count},
        {"endpoint_infeasible_count", either_collision_count},
        {"start_collision_count", start_collision_count},
        {"goal_collision_count", goal_collision_count},
        {"both_collision_count", both_collision_count},
        {"check_time_ms", elapsed}
    };

    const std::filesystem::path output_path(output_value);
    std::error_code ec;
    std::filesystem::create_directories(output_path.parent_path(), ec);
    if (ec) {
        throw std::runtime_error("Failed to create endpoint collision output directory: " + ec.message());
    }
    std::ofstream output(output_path);
    if (!output.is_open()) {
        throw std::runtime_error("Failed to open endpoint collision output: " + output_path.string());
    }
    output << std::setw(2) << report << std::endl;
    std::cout << "Endpoint collision check complete: " << either_collision_count
              << "/" << pair_count << " tasks are infeasible; output="
              << output_path << std::endl;
    return true;
}
}
std::vector<const Node3D*> TracePath(const Node3D* node, int i, std::vector<const Node3D*>& path) {
  if (node == nullptr) { return path; }
  path.push_back(node);
  return TracePath(node->getPred(), i, path);
}

Planner::Planner() {

  if (Constants::manual) {
    subMap = n.subscribe("/map", 1, &Planner::setMap, this);
  } else {
    subMap = n.subscribe("/occ_map", 1, &Planner::setMap, this);
  }

  subGoal = n.subscribe("/move_base_simple/goal", 1, &Planner::setGoal, this);
  subStart = n.subscribe("/initialpose", 1, &Planner::setStart, this);

};

void Planner::initializeLookups() {
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubinsLookup);
  }

  Lookup::collisionLookup(collisionLookup);
}

void Planner::setMap(const nav_msgs::OccupancyGrid::Ptr map) {
  if (Constants::coutDEBUG) {
    std::cout << "I am seeing the map..." << std::endl;
  }

  grid = map;

  configurationSpace.updateGrid(map);

  int height = map->info.height;
  int width = map->info.width;
  bool** binMap;
  binMap = new bool*[width];

  for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }

  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      binMap[x][y] = map->data[y * width + x] ? true : false;
    }
  }

  voronoiDiagram.initializeMap(width, height, binMap);
  voronoiDiagram.update();
  voronoiDiagram.visualize();

  if (!Constants::manual && listener.canTransform("/map", ros::Time(0), "/base_link", ros::Time(0), "/map", nullptr)) {

    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

    start.pose.pose.position.x = transform.getOrigin().x();
    start.pose.pose.position.y = transform.getOrigin().y();
    tf::quaternionTFToMsg(transform.getRotation(), start.pose.pose.orientation);

    if (grid && grid->info.height >= start.pose.pose.position.y && start.pose.pose.position.y >= 0 &&
        grid->info.width >= start.pose.pose.position.x && start.pose.pose.position.x >= 0) {

      validStart = true;
    } else  {
      validStart = false;
    }

    plan();
  }
}

void Planner::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {
  float x = initial->pose.pose.position.x / Constants::cellSize;
  float y = initial->pose.pose.position.y / Constants::cellSize;
  float t = tf::getYaw(initial->pose.pose.orientation);

  geometry_msgs::PoseStamped startN;
  startN.pose.position = initial->pose.pose.position;
  startN.pose.orientation = initial->pose.pose.orientation;
  startN.header.frame_id = "map";
  startN.header.stamp = ros::Time::now();

  std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid && grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validStart = true;
    start = *initial;

    if (Constants::manual) { plan();}

  } else {
    std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

void Planner::setGoal(const geometry_msgs::PoseStamped::ConstPtr& end) {

  float x = end->pose.position.x / Constants::cellSize;
  float y = end->pose.position.y / Constants::cellSize;
  float t = tf::getYaw(end->pose.orientation);

  geometry_msgs::PoseStamped goalN;
  goalN.pose.position = end->pose.position;
  goalN.pose.orientation = end->pose.orientation;
  goalN.header.frame_id = "map";
  goalN.header.stamp = ros::Time::now();

  std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid && grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validGoal = true;
    goal = *end;

    if (Constants::manual) { plan();}

  } else {
    std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

void loadAndDisplayPathFromJson(const std::string& json_file_path,
                                  Path& path,
                                  CollisionDetection& configurationSpace) {

    std::ifstream json_file(json_file_path);
    if (!json_file.is_open()) {
        std::cerr << "❌ Cannot open JSON file: " << json_file_path << std::endl;
        return;
    }

    try {

        json j_data;
        json_file >> j_data;

        if (!j_data.is_array()) {
            std::cerr << "❌ JSON file should contain an array" << std::endl;
            return;
        }

        std::cout << "📂 Loaded " << j_data.size() << " path(s) from: " << json_file_path << std::endl;

        for (size_t idx = 0; idx < j_data.size(); idx++) {
            const json& j_path = j_data[idx];

            if (!j_path.contains("path_points") || !j_path["path_points"].is_array()) {
                std::cout << "⚠️  Path index " << idx << " has no path_points, skipping..." << std::endl;
                continue;
            }

            if (j_path.contains("start") && j_path.contains("goal")) {
                double start_x = j_path["start"]["x"];
                double start_y = j_path["start"]["y"];
                double start_yaw = j_path["start"]["yaw"];
                double goal_x = j_path["goal"]["x"];
                double goal_y = j_path["goal"]["y"];
                double goal_yaw = j_path["goal"]["yaw"];

                start_yaw = Helper::normalizeHeadingRad(start_yaw);
                goal_yaw = Helper::normalizeHeadingRad(goal_yaw);

                Node3D nStart(start_x, start_y, start_yaw, 0, 0, nullptr);
                Node3D nGoal(goal_x, goal_y, goal_yaw, 0, 0, nullptr);

                path.addVehicle_SPMT_earcut(nStart, 10000 + idx, false,false, HybridAStar::PathColor(1.0, 0.0, 0.0, 0.7));
                path.addVehicle_SPMT_earcut(nGoal, 20000 + idx, false,false, HybridAStar::PathColor(0.0, 1.0, 0.0, 0.7));
            }

            std::vector<Node3D> path_objects;
            const json& j_points = j_path["path_points"];

            for (const auto& j_point : j_points) {
                double x = j_point["x"];
                double y = j_point["y"];
                double theta = j_point["theta"];

                path_objects.emplace_back(x, y, theta, 0, 0, nullptr);
            }

            if (!path_objects.empty()) {
                path.updatePath(path_objects);
                std::cout << "✅ Displayed path " << idx << " with " << path_objects.size() << " points" << std::endl;
            }
        }

        path.publishPathVehicles();

    } catch (const std::exception& e) {
        std::cerr << "❌ Error parsing JSON: " << e.what() << std::endl;
    }
}

void example_loadAndDisplayAntGap14(Path& path, CollisionDetection& configurationSpace) {
    std::string json_file = "data/ant-gap14.json";
    loadAndDisplayPathFromJson(json_file, path, configurationSpace);
}

static HybridAStar::PathColor hsvToRgb(float h, float s, float v) {
    float c = v * s;
    float x = c * (1 - std::abs(fmod(h / 60.0f, 2) - 1));
    float m = v - c;

    float r, g, b;
    if (h < 60) {
        r = c; g = x; b = 0;
    } else if (h < 120) {
        r = x; g = c; b = 0;
    } else if (h < 180) {
        r = 0; g = c; b = x;
    } else if (h < 240) {
        r = 0; g = x; b = c;
    } else if (h < 300) {
        r = x; g = 0; b = c;
    } else {
        r = c; g = 0; b = x;
    }

    return HybridAStar::PathColor(r + m, g + m, b + m, 0.8);
}

void loadAndDisplayPathPairs(const std::string& json_file_path,
                              Path& path,
                              CollisionDetection& configurationSpace) {

    std::ifstream json_file(json_file_path);
    if (!json_file.is_open()) {
        std::cerr << "Cannot open JSON file: " << json_file_path << std::endl;
        return;
    }

    try {

        json j_data;
        json_file >> j_data;

        if (!j_data.is_array()) {
            std::cerr << "JSON file should contain an array" << std::endl;
            return;
        }

        std::cout << "Loaded " << j_data.size() << " path pairs from: " << json_file_path << std::endl;

        int feasible_count = 0;
        int infeasible_count = 0;
        std::vector<double> radius_finded={

        10,20, 30, 40,60, 70, 80,

        90,110, 120, 130, 140,

        160, 170, 180, 190, 210, 220, 230,

         240
        };
        int updatePathWithColor_tip=1000000;

        for (size_t idx = 0; idx < j_data.size(); idx++) {
            const json& j_path = j_data[idx];

            if (!j_path.contains("start") || !j_path.contains("goal")) {
                std::cout << "Path index " << idx << " missing start or goal, skipping..." << std::endl;
                continue;
            }

            double start_x = j_path["start"]["x"];
            double start_y = j_path["start"]["y"];
            double start_yaw = j_path["start"]["yaw"];
            double goal_x = j_path["goal"]["x"];
            double goal_y = j_path["goal"]["y"];
            double goal_yaw = j_path["goal"]["yaw"];

            double radius = j_path["test_info"]["radius"];
            bool already_exist=false;
            for (const auto& r : radius_finded) {
                if (r == radius) {
                    already_exist = true;
                }
            }
            if(already_exist==false){
                radius_finded.push_back(radius);
            }

            start_yaw = Helper::normalizeHeadingRad(start_yaw);
            goal_yaw = Helper::normalizeHeadingRad(goal_yaw);

            Node3D nStart(start_x, start_y, start_yaw, 0, 0, nullptr);
            Node3D nGoal(goal_x, goal_y, goal_yaw, 0, 0, nullptr);

            bool is_feasible = false;
            if (j_path.contains("feasible") && j_path["feasible"].is_object()) {
                const auto& feasible = j_path["feasible"];

                is_feasible = (feasible.contains("astar") && feasible["astar"].is_boolean() && feasible["astar"].get<bool>()) ||
                              (feasible.contains("dsu") && feasible["dsu"].is_boolean() && feasible["dsu"].get<bool>()) ||
                              (feasible.contains("searching") && feasible["searching"].is_boolean() && feasible["searching"].get<bool>());
            }

            float hue = fmod((idx * 137.508f), 360.0f);
            HybridAStar::PathColor pair_color = hsvToRgb(hue, 0.7f, 1.0f);

                if(already_exist==false){

                    path.addVehicle_SPMT_earcut(nStart, 10000 + idx * 2, false, true,pair_color);
                    path.addVehicle_SPMT_earcut(nGoal, 10001 + idx * 2, false,true, pair_color);
                }else{

                    path.addVehicle_TriangleMarker(nStart, 10000 + idx * 2, false, pair_color);
                    path.addVehicle_TriangleMarker(nGoal, 10001 + idx * 2, false, pair_color);
                }

        }

        std::cout << "Display complete: " << feasible_count << " feasible paths, "
                  << infeasible_count << " infeasible paths (gray lines)" << std::endl;

        path.publishPathVehicles();

    } catch (const std::exception& e) {
        std::cerr << "Error parsing JSON: " << e.what() << std::endl;
    }
}

void example_loadAndDisplayOnsite450Pair(Path& path, CollisionDetection& configurationSpace) {
    std::string json_file = "";
    loadAndDisplayPathPairs(json_file, path, configurationSpace);
}

int tip = 0;

void Planner::plan() {

#ifdef ONSITE_show_radom_sample

    Path path;
    example_loadAndDisplayOnsite450Pair(path, configurationSpace);
#endif

  if (validStart && validGoal && grid) {
    std::cout << "goal and start are getted !" << std::endl;
    std::cout << "Start : " << validStart<<std::endl;
    std::cout << "Goal : "  << validGoal<<std::endl;

    int width = grid->info.width;
    int height = grid->info.height;

#ifdef PARKING
    const std::vector<int>& selected_parking_indices = selectedParkingIndices();
    const int parking_pair_count = selected_parking_indices.empty()
        ? static_cast<int>(HybridAStar::Constants::parking_plot().size())
        : static_cast<int>(selected_parking_indices.size());
    if (!HybridAStar::Constants::Experiment_Run_One_Pair && tip == 0) {
      const char* resume_from_env = std::getenv("HYBRID_ASTAR_RESUME_FROM");
      const char* resume_results_env = std::getenv("HYBRID_ASTAR_RESUME_RESULTS");
      if (resume_from_env != nullptr && resume_results_env != nullptr) {
        const int requested_resume = std::max(0, std::atoi(resume_from_env));
        tip = loadExistingResultsFromJson(
            sg_pair,
            resume_results_env,
            std::min(requested_resume, parking_pair_count));
      }
    }
    if (!HybridAStar::Constants::Experiment_Run_One_Pair &&
        tip >= parking_pair_count) {
      validStart = false;
      validGoal = false;
      return;
    }
#endif

#ifdef ONSITE
    const std::vector<int>& selected_onsite_indices = selectedOnsiteIndices();
#endif

#ifdef ONSITE
    if (!HybridAStar::Constants::Experiment_Run_One_Pair &&
        HybridAStar::Constants::Experiment_Use_Onsite_450_Pairs &&
        !sg_pair_finded) {
      num_pairs = loadOnsitePairsFromJson(sg_pair, HybridAStar::Constants::Experiment_Onsite_450_Pairs_File);
      if (!selected_onsite_indices.empty()) {
        std::vector<HybridAStar::Constants::Pathpair> selected_pairs;
        selected_pairs.reserve(selected_onsite_indices.size());
        for (const int index : selected_onsite_indices) {
          selected_pairs.push_back(sg_pair[index]);
        }
        for (size_t i = 0; i < selected_pairs.size(); ++i) {
          sg_pair[i] = selected_pairs[i];
        }
        num_pairs = static_cast<int>(selected_onsite_indices.size());
      }
      const char* resume_from_env = std::getenv("HYBRID_ASTAR_RESUME_FROM");
      const char* resume_results_env = std::getenv("HYBRID_ASTAR_RESUME_RESULTS");
      if (resume_from_env != nullptr && resume_results_env != nullptr) {
        const int requested_resume = std::max(0, std::atoi(resume_from_env));
        tip = loadExistingResultsFromJson(
            sg_pair,
            resume_results_env,
            std::min(requested_resume, num_pairs));
      }
      sg_pair_finded = true;
    }
#endif

#ifdef ONSITE
    if (!HybridAStar::Constants::Experiment_Run_One_Pair &&
        HybridAStar::Constants::Experiment_Use_Onsite_450_Pairs &&
        tip >= num_pairs) {
      validStart = false;
      validGoal = false;
      return;
    }
#endif

#ifdef ONSITE
    if (runOnsiteEndpointCollisionCheck(sg_pair, num_pairs, configurationSpace)) {
      validStart = false;
      validGoal = false;
      return;
    }
#endif

#ifdef ONSITE_test_radom_sample

    if(!sg_pair_finded){
      num_pairs = find_sg_pair(sg_pair, width, height);
      sg_pair_finded=true;
    }
    printf("sg_pair[%d].radus = %d\r\n",tip,sg_pair[tip].radi_test);
#endif

#ifndef AWS_from_others
    int depth = Constants::headings;

    int lengthM = width * height * depth * Constants::positions;
    Node3D* nodes3DM = new Node3D[lengthM]();
#endif

#ifdef AWS_from_others
    int depth = Constants_aws::headings;

    int lengthM = width * height * depth * Constants_aws::positions;
    Node2D* nodes2D = new Node2D[width * height]();
    NodeAWS* nodes3D_aws = new NodeAWS[lengthM]();
#endif

    float x = goal.pose.position.x / Constants::cellSize;
    float y = goal.pose.position.y / Constants::cellSize;
    float t = tf::getYaw(goal.pose.orientation);

#ifdef PARKING
    const int parking_target_index = selected_parking_indices.empty()
        ? tip
        : selected_parking_indices[tip];
    if(parking_target_index < static_cast<int>(HybridAStar::Constants::parking_plot().size()))
    {
      x=HybridAStar::Constants::parking_plot()[parking_target_index][0];
      y=HybridAStar::Constants::parking_plot()[parking_target_index][1];
      t=HybridAStar::Constants::parking_plot()[parking_target_index][2];
    }
#endif

#ifdef ANT
      x=HybridAStar::Constants::ant_sg()[1][0];
      y=HybridAStar::Constants::ant_sg()[1][1];
      t=HybridAStar::Constants::ant_sg()[1][2];
#endif

#ifdef uturn_cross
      x=HybridAStar::Constants::uturn_cross_sg()[1][0];
      y=HybridAStar::Constants::uturn_cross_sg()[1][1];
      t=HybridAStar::Constants::uturn_cross_sg()[1][2];
#endif

#ifdef uturn_green_sg2
      x=HybridAStar::Constants::uturn_green_sg()[2][0];
      y=HybridAStar::Constants::uturn_green_sg()[2][1];
      t=HybridAStar::Constants::uturn_green_sg()[2][2];
#endif

#ifdef ONSITE
      if (!HybridAStar::Constants::Experiment_Run_One_Pair &&
          HybridAStar::Constants::Experiment_Use_Onsite_450_Pairs && tip < num_pairs) {
        x = sg_pair[tip].goal_x;
        y = sg_pair[tip].goal_y;
        t = sg_pair[tip].goal_yaw;
      } else {
        x=HybridAStar::Constants::onsite_sg()[1][0];
        y=HybridAStar::Constants::onsite_sg()[1][1];
        t=HybridAStar::Constants::onsite_sg()[1][2];
      }
#endif

    t = Helper::normalizeHeadingRad(t);
    const Node3D nGoal(x, y, t, 0, 0, nullptr);

#ifdef AWS_from_others
    const NodeAWS nGoal_aws(nGoal);
#endif

    x = start.pose.pose.position.x / Constants::cellSize;
    y = start.pose.pose.position.y / Constants::cellSize;
    t = tf::getYaw(start.pose.pose.orientation);

#ifdef PARKING

#endif

#ifdef ANT
      x=HybridAStar::Constants::ant_sg()[0][0];
      y=HybridAStar::Constants::ant_sg()[0][1];
      t=HybridAStar::Constants::ant_sg()[0][2];
#endif

#ifdef uturn_cross
      x=HybridAStar::Constants::uturn_cross_sg()[0][0];
      y=HybridAStar::Constants::uturn_cross_sg()[0][1];
      t=HybridAStar::Constants::uturn_cross_sg()[0][2];
#endif

#ifdef uturn_green_sg2
      x=HybridAStar::Constants::uturn_green_sg()[0][0];
      y=HybridAStar::Constants::uturn_green_sg()[0][1];
      t=HybridAStar::Constants::uturn_green_sg()[0][2];
#endif
#ifdef PARKING
      x=11;
      y=94;
      t=0;
#endif
#ifdef ONSITE
      if (!HybridAStar::Constants::Experiment_Run_One_Pair &&
          HybridAStar::Constants::Experiment_Use_Onsite_450_Pairs && tip < num_pairs) {
        x = sg_pair[tip].start_x;
        y = sg_pair[tip].start_y;
        t = sg_pair[tip].start_yaw;
      } else {
        x=HybridAStar::Constants::onsite_sg()[0][0];
        y=HybridAStar::Constants::onsite_sg()[0][1];
        t=HybridAStar::Constants::onsite_sg()[0][2];
      }
#endif

#ifdef obca_output
      x=5;
      y=5;
      t=0;
#endif

    t = Helper::normalizeHeadingRad(t);
    Node3D nStart(x, y, t, 0, 0, nullptr);

#ifdef AWS_from_others
    NodeAWS nStart_aws(nStart);
#endif

    visualization.clear();

    path.clear();

#ifdef uturn_green_sg

    x=HybridAStar::Constants::uturn_green_sg()[0][0];
    y=HybridAStar::Constants::uturn_green_sg()[0][1];
    t=HybridAStar::Constants::uturn_green_sg()[0][2];
    t = Helper::normalizeHeadingRad(t);
    Node3D p1(x, y, t, 0, 0, nullptr);

    x=HybridAStar::Constants::uturn_green_sg()[1][0];
    y=HybridAStar::Constants::uturn_green_sg()[1][1];
    t=HybridAStar::Constants::uturn_green_sg()[1][2];
    t = Helper::normalizeHeadingRad(t);
    Node3D p2(x, y, t, 0, 0, nullptr);

    x=HybridAStar::Constants::uturn_green_sg()[2][0];
    y=HybridAStar::Constants::uturn_green_sg()[2][1];
    t=HybridAStar::Constants::uturn_green_sg()[2][2];
    t = Helper::normalizeHeadingRad(t);
    Node3D p3(x, y, t, 0, 0, nullptr);

    path.addVehicle_SPMT_earcut(p1,10000,false, HybridAStar::PathColor(0.0, 1.0, 0.0, 0.7));
    path.addVehicle_SPMT_earcut(p2,10001,false, HybridAStar::PathColor(1.0, 0.0, 0.0, 0.7));
    path.addVehicle_SPMT_earcut(p3,10001,false, HybridAStar::PathColor(0.0, 0.0, 1.0, 0.7));
#endif

#ifdef show_sg

    path.addVehicle_SPMT_earcut_forever(nStart,10000,false, HybridAStar::PathColor(0.0, 1.0, 0.0, 0.7));
    path.addVehicle_SPMT_earcut_forever(nGoal,10001,false, HybridAStar::PathColor(0.0, 0.0, 1.0, 0.7));
#endif
#ifdef show_car_occupied

    std::vector<Eigen::Vector2d> obs_start=configurationSpace.returnpolgen(nStart.getX(),nStart.getY(),nStart.getT(),true);

    path.add_occupy(obs_start,1);

#endif
#ifdef test_sg

    validStart=0;
    validGoal=0;
    return;
#endif

    bool path_is_feasible = false;

    double t_judge_output_Astar =0.0;
    double t_judge_output_DSU   =0.0;
    double t_planning_output    =0.0;
    int planning_iterations = 0;
    double path_cost = 0.0;

    bool path_is_feasible_Astar    = false;
    bool path_is_feasible_DSU      = false;
    bool path_is_feasible_searching= false;

    int length;

#ifndef AWS_from_others
    Node3D* nSolution = Algorithm::hybridAStar_M(nStart,nGoal,nodes3DM, width, height, configurationSpace, dubinsLookup,
                                                  &dsuParent, &dsuRank, &dsuInitialized,
                                                  &t_judge_output_Astar, &t_judge_output_DSU,&t_planning_output,
                                                &path_is_feasible_Astar,&path_is_feasible_DSU,&path_is_feasible_searching,
                                                &planning_iterations, &path_cost);
#endif

#ifdef AWS_from_others
    const auto planning_start = std::chrono::steady_clock::now();
    Node3D* nSolution = Algorithm::hybridAStar_aws(nStart_aws, nGoal_aws, nodes3D_aws, nodes2D, width, height, configurationSpace, dubinsLookup, visualization,false);
    const auto planning_end = std::chrono::steady_clock::now();
    t_planning_output = std::chrono::duration<double, std::milli>(planning_end - planning_start).count();
    path_is_feasible_searching = (nSolution != nullptr);
#endif

    std::vector<const Node3D*> path_p;

    TracePath(nSolution, 0, path_p);
    std::cout<<"path_p.size(): "<<path_p.size()<<std::endl;

    std::vector<Node3D> path_objects;
    path_objects.reserve(path_p.size());
    for (const Node3D* node_ptr : path_p) {
        if (node_ptr != nullptr) {
            path_objects.insert(path_objects.begin(), *node_ptr);
        }
    }
    path_objects.push_back(nGoal);

#ifdef show_traj

    path.updatePath(path_objects);
#endif

    sg_pair[tip].start_x = nStart.getX();
    sg_pair[tip].start_y = nStart.getY();
    sg_pair[tip].start_yaw = nStart.getT();
    sg_pair[tip].goal_x = nGoal.getX();
    sg_pair[tip].goal_y = nGoal.getY();
    sg_pair[tip].goal_yaw = nGoal.getT();
    sg_pair[tip].t_judge_output_Astar = t_judge_output_Astar;
    sg_pair[tip].t_judge_output_DSU = t_judge_output_DSU;
    sg_pair[tip].t_planning_output = t_planning_output;
    sg_pair[tip].planning_iterations = planning_iterations;
    sg_pair[tip].path_cost = path_cost;
    sg_pair[tip].path_is_feasible_Astar = path_is_feasible_Astar;
    sg_pair[tip].path_is_feasible_DSU = path_is_feasible_DSU;
    sg_pair[tip].path_is_feasible_searching = path_is_feasible_searching;
    sg_pair[tip].length_t = path_objects.size()-2 ;

    if (nSolution != nullptr && path_objects.size() > 1) {
        double path_length = 0.0;
        for (size_t i = 0; i < path_objects.size() - 1; i++) {
            double dx = path_objects[i+1].getX() - path_objects[i].getX();
            double dy = path_objects[i+1].getY() - path_objects[i].getY();
            path_length += std::sqrt(dx*dx + dy*dy);
        }
        sg_pair[tip].path_length = path_length;
    } else {
        sg_pair[tip].path_length = 0;
    }

    sg_pair[tip].path_xyt.clear();
    for (const auto& node : path_objects) {
        sg_pair[tip].path_xyt.push_back(std::make_tuple(node.getX(), node.getY(), node.getT()));
    }
    sg_pair[tip].result_valid = true;

    tip++;

    namespace fs = std::filesystem;
    const fs::path experiment_dir = fs::path(HybridAStar::Constants::Experiment_Output_Root) /
                                    HybridAStar::Constants::Experiment_Method /
                                    HybridAStar::Constants::Experiment_Scene /
                                    HybridAStar::Constants::Experiment_Map /
                                    "path_pairs";
    std::error_code ec;
    fs::create_directories(experiment_dir, ec);
    if (ec) {
        std::cerr << "Failed to create experiment output dir: " << experiment_dir << " (" << ec.message() << ")" << std::endl;
    }

    const std::string result_prefix = "path_pairs_" + HybridAStar::Constants::Experiment_Method + "_" +
                                      HybridAStar::Constants::Experiment_Scene + "_" +
                                      HybridAStar::Constants::Experiment_Map;
    int result_file_index = tip;
#ifdef PARKING
    if (!selected_parking_indices.empty() && tip > 0 &&
        tip <= static_cast<int>(selected_parking_indices.size())) {
        result_file_index = selected_parking_indices[tip - 1] + 1;
    }
#endif
#ifdef ONSITE
    if (!selected_onsite_indices.empty() && tip > 0 &&
        tip <= static_cast<int>(selected_onsite_indices.size())) {
        result_file_index = selected_onsite_indices[tip - 1] + 1;
    }
#endif
    std::stringstream backup_name;
    backup_name << result_prefix << "_backup_" << std::setfill('0') << std::setw(5)
                << result_file_index << ".json";
    std::string filename = (experiment_dir / backup_name.str()).string();

    try {

        json j_array = json::array();

        for (int i = 0; i < tip && i < 560; i++) {
            if (!sg_pair[i].result_valid) {
                continue;
            }

            json j;

            j["start"] = {
                {"x", sg_pair[i].start_x},
                {"y", sg_pair[i].start_y},
                {"yaw", sg_pair[i].start_yaw}
            };

            j["goal"] = {
                {"x", sg_pair[i].goal_x},
                {"y", sg_pair[i].goal_y},
                {"yaw", sg_pair[i].goal_yaw}
            };

            j["path_length"] = sg_pair[i].path_length;
            j["path_cost"] = sg_pair[i].path_cost;

            j["length_t"] = sg_pair[i].length_t;

            j["time"] = {
                {"judge_astar", sg_pair[i].t_judge_output_Astar},
                {"judge_dsu", sg_pair[i].t_judge_output_DSU},
                {"planning", sg_pair[i].t_planning_output},
                {"planning_iterations", sg_pair[i].planning_iterations}
            };

            j["feasible"] = {
                {"astar", sg_pair[i].path_is_feasible_Astar},
                {"dsu", sg_pair[i].path_is_feasible_DSU},
                {"searching", sg_pair[i].path_is_feasible_searching}
            };

            j["test_info"] = {
                {"radius", sg_pair[i].radi_test}
            };
#ifdef PARKING
            j["test_info"]["parking_index"] = selected_parking_indices.empty()
                ? i + 1
                : selected_parking_indices[i] + 1;
#endif
#ifdef ONSITE
            j["test_info"]["onsite_index"] = selected_onsite_indices.empty()
                ? i + 1
                : selected_onsite_indices[i] + 1;
#endif

            j["path_points"] = json::array();
            for (const auto& point : sg_pair[i].path_xyt) {
                double x, y, theta;
                std::tie(x, y, theta) = point;
                j["path_points"].push_back({
                    {"x", x},
                    {"y", y},
                    {"theta", theta}
                });
            }

            j_array.push_back(j);
        }

        std::ofstream file(filename);
        if (file.is_open()) {
            file << std::setw(4) << j_array << std::endl;
            file.close();
            std::cout << "✅ Saved valid path pairs up to tip " << tip << " to: " << filename << std::endl;
        } else {
            std::cerr << "❌ Failed to open file: " << filename << std::endl;
        }

        const std::string latest_filename = (experiment_dir / (result_prefix + "_latest.json")).string();
        std::ofstream file_latest(latest_filename);
        if (file_latest.is_open()) {
            file_latest << std::setw(4) << j_array << std::endl;
            file_latest.close();
        }
    } catch (const std::exception& e) {
        std::cerr << "❌ Error saving JSON: " << e.what() << std::endl;
    }

#ifdef show_traj
    path.publishPathVehicles();
#endif

    validStart=false;
    validGoal=false;

#ifndef AWS_from_others
    delete [] nodes3DM;
#endif

#ifdef AWS_from_others
    Algorithm::releaseAwsSearchMemory();
    delete [] nodes2D;
    delete [] nodes3D_aws;
#endif

#ifdef PARKING
    if (!HybridAStar::Constants::Experiment_Run_One_Pair) {
        if (tip >= parking_pair_count) {
            validStart=false;
            validGoal=false;
            return;
        }
        validStart=true;
        validGoal =true;
        plan();
    }
#endif

#ifdef ONSITE
    if (!HybridAStar::Constants::Experiment_Run_One_Pair) {
        if (tip >= num_pairs) {
            validStart=false;
            validGoal=false;
            return;
        }
        validStart=true;
        validGoal =true;
        plan();
    }
#endif

  } else {

    if(validStart==false){
      std::cout << "Missing Start " <<std::endl;
      }
      if(validGoal==false){
      std::cout << "Missing Goal " <<std::endl;
      }
    }
}

inline double calculateConfigDistance(double x1, double y1, double yaw1,
                                       double x2, double y2, double yaw2) {
    double dx = x1 - x2;
    double dy = y1 - y2;
    double dyaw = yaw1 - yaw2;

    while (dyaw > M_PI) dyaw -= 2 * M_PI;
    while (dyaw < -M_PI) dyaw += 2 * M_PI;

    double yaw_weight = HybridAStar::Constants::V_max_vision / HybridAStar::Constants::w_max_wheel;
    return std::sqrt(dx * dx + dy * dy + dyaw * dyaw * yaw_weight * yaw_weight);
}

int Planner::find_sg_pair(Constants::Pathpair* pair, int width, int height, bool enable_visualization) {

    if (width <= 0 || height <= 0) {
        std::cerr << "Error: Invalid width or height in find_sg_pair! width="
                  << width << ", height=" << height << std::endl;
        return 0;
    }

    if (!this->grid) {
        std::cerr << "Error: grid is not initialized in find_sg_pair!" << std::endl;
        return 0;
    }

    std::vector<double> radii = HybridAStar::Constants::onsite_radom_length();
    int pairs_per_radius = HybridAStar::Constants::pairs_per_radius;
    int max_attempts = 100000;

    std::cout << "\n========================================" << std::endl;
    std::cout << "  Generating Start-Goal Pairs" << std::endl;
    std::cout << "  Map size: " << width << "x" << height << std::endl;
    std::cout << "========================================" << std::endl;

    int total_pairs = 0;
    int pair_index = 0;

    std::srand(std::time(nullptr));

    for (double radius : radii) {
        std::cout << "\nGenerating pairs for radius: " << radius << " meters" << std::endl;

        int valid_pairs_for_radius = 0;
        int attempts = 0;

        while (valid_pairs_for_radius < pairs_per_radius && attempts < max_attempts) {
            attempts++;

            double start_x = ((double)std::rand() / RAND_MAX) * width;
            double start_y = ((double)std::rand() / RAND_MAX) * height;
            double start_yaw = ((double)std::rand() / RAND_MAX) * 2 * M_PI;

            if (!configurationSpace.configurationTest_polygon(start_x, start_y, start_yaw)) {
                continue;
            }

            double goal_yaw = ((double)std::rand() / RAND_MAX) * 2 * M_PI;
            double dyaw_yaw = std::abs(goal_yaw-start_yaw);
            if(dyaw_yaw > M_PI){
              dyaw_yaw = dyaw_yaw - M_PI;
            }
            double yaw_scale = HybridAStar::Constants::V_max_vision / HybridAStar::Constants::w_max_wheel;
            double dyaw_yaw_t=dyaw_yaw / yaw_scale;
            double remain_l=std::sqrt(radius*radius-dyaw_yaw_t*dyaw_yaw_t);
            double dx =((double)std::rand() / RAND_MAX) *remain_l;
            double dy =std::sqrt(remain_l*remain_l-dx*dx);

            int sign_x = (std::rand() % 2 == 0) ? 1 : -1;
            int sign_y = (std::rand() % 2 == 0) ? 1 : -1;
            dx = dx * sign_x;
            dy = dy * sign_y;

            double goal_x = start_x + dx;
            double goal_y = start_y + dy;

            if (goal_x < 0 || goal_x >= width || goal_y < 0 || goal_y >= height) {
                continue;
            }

            if (!configurationSpace.configurationTest_polygon(goal_x, goal_y, goal_yaw)) {
                continue;
            }

            bool is_valid = true;
            for (int i = 0; i < total_pairs; i++) {
                const Constants::Pathpair& existing = pair[i];

                double d_ss = calculateConfigDistance(start_x, start_y, start_yaw,
                                                      existing.start_x, existing.start_y, existing.start_yaw);

                double d_ee = calculateConfigDistance(goal_x, goal_y, goal_yaw,
                                                      existing.goal_x, existing.goal_y, existing.goal_yaw);

                double d_se = calculateConfigDistance(start_x, start_y, start_yaw,
                                                      existing.goal_x, existing.goal_y, existing.goal_yaw);

                double d_es = calculateConfigDistance(goal_x, goal_y, goal_yaw,
                                                      existing.start_x, existing.start_y, existing.start_yaw);

                if ((d_ss < 1.0 && d_ee < 1.0) || (d_se < 1.0 && d_es < 1.0)) {
                    is_valid = false;
                    break;
                }
            }

            if (!is_valid) {
                continue;
            }

            pair[pair_index].start_x = start_x;
            pair[pair_index].start_y = start_y;
            pair[pair_index].start_yaw = start_yaw;
            pair[pair_index].goal_x = goal_x;
            pair[pair_index].goal_y = goal_y;
            pair[pair_index].goal_yaw = goal_yaw;
            pair[pair_index].radi_test = radius;

            pair_index++;
            total_pairs++;
            valid_pairs_for_radius++;

            if (valid_pairs_for_radius % 10 == 0) {
                std::cout << "  Generated " << valid_pairs_for_radius
                          << "/" << pairs_per_radius << " pairs (attempts: " << attempts << ")" << std::endl;
            }

            if (enable_visualization) {

            }
        }

        if (valid_pairs_for_radius < pairs_per_radius) {
            std::cout << "  Warning: Only generated " << valid_pairs_for_radius
                      << "/" << pairs_per_radius << " pairs for radius " << radius << std::endl;
        } else {
            std::cout << "  Successfully generated " << valid_pairs_for_radius
                      << " pairs for radius " << radius << std::endl;
        }

    }

    std::cout << "\n========================================" << std::endl;
    std::cout << "  Total pairs generated: " << total_pairs << std::endl;
    std::cout << "========================================\n" << std::endl;

    return total_pairs;
    num_pairs=total_pairs;
}
