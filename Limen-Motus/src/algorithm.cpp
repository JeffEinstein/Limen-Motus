#include "algorithm.h"
#include <boost/heap/binomial_heap.hpp>
#include <memory>
#include <vector>
#include "Sampling_M.h"
#include <chrono>

using namespace HybridAStar;

struct Color {
    float r, g, b, a;
    Color(float r = 0.0, float g = 0.0, float b = 0.0, float a = 0.1)
        : r(r), g(g), b(b), a(a) {}
};

const Color RED(1.0, 0.0, 0.0, 1.0);
const Color BLUE(0.0, 0.0, 1.0, 1.0);
const Color GREEN(0.0, 1.0, 0.0, 1.0);
const Color YELLOW(1.0, 1.0, 0.0, 1.0);
const Color PURPLE(1.0, 0.0, 1.0, 1.0);
const Color CYAN(0.0, 1.0, 1.0, 1.0);
const Color WHITE(1.0, 1.0, 1.0, 1.0);
const Color BLACK(0.0, 0.0, 0.0, 1.0);

ros::Publisher marker_pub;
ros::Publisher cset_pub;

Path_plot plt_M;

namespace {
std::vector<std::unique_ptr<NodeAWS>> awsSuccessorPool;
}

void Algorithm::releaseAwsSearchMemory() {
  std::vector<std::unique_ptr<NodeAWS>>().swap(awsSuccessorPool);
}

void initPathPublisher(ros::NodeHandle& nh) {

    marker_pub = nh.advertise<visualization_msgs::Marker>("temp_xyt", 10);
    cset_pub = nh.advertise<visualization_msgs::Marker>("close_set", 10);

    ros::Duration(0.1).sleep();
    ros::spinOnce();
}

visualization_msgs::Marker createPointMarker(
    const Node3D& Node,
    const Color& color,
    int id,
    const std::string& ns ,
    float size = 1.0,
    const std::string& frame_id = "map"
) {

    geometry_msgs::Point point;
    point.x = Node.getX();
    point.y = Node.getY();
    point.z = Node.getT();

    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.points.push_back(point);

    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = 0.0;

    marker.color.r = color.r;
    marker.color.g = color.g;
    marker.color.b = color.b;
    marker.color.a = color.a;

    marker.lifetime = ros::Duration(0);

    return marker;
}

void publishMarker(
    const Node3D& Node,
    const Color& color,
    int id,
    const std::string& ns ,
    float size = 1.0
) {
    visualization_msgs::Marker marker = createPointMarker(Node, color, id, ns, size);
    marker_pub.publish(marker);
    ros::Duration(0.1).sleep();

}

void publishMarker_openset(
    const Node3D& Node,
    const Color& color,
    int id,
    const std::string& ns ,
    float size = 1.0
) {
    visualization_msgs::Marker marker = createPointMarker(Node, color, id, ns, size);
    cset_pub.publish(marker);
    ros::Duration(0.1).sleep();

}

void clearMarkers(const std::string& ns) {
    visualization_msgs::Marker clear_marker;
    clear_marker.header.frame_id = "map";
    clear_marker.header.stamp = ros::Time::now();
    clear_marker.ns = ns;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_pub.publish(clear_marker);
    ros::Duration(0.1).sleep();
    ros::spinOnce();
}

void clearMarkers_openset(const std::string& ns) {
    visualization_msgs::Marker clear_marker;
    clear_marker.header.frame_id = "map";
    clear_marker.header.stamp = ros::Time::now();
    clear_marker.ns = ns;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    cset_pub.publish(clear_marker);
    ros::Duration(0.1).sleep();
    ros::spinOnce();
}

NodeAWS* dubinsShot(NodeAWS& start, const NodeAWS& goal, CollisionDetection& configurationSpace);
std::vector<std::vector<float>> AWSConstructSamplingTable(int r_samples, int psi_samples, std::vector<float> omega_sample_list = {M_PI_2, M_PI/3, M_PI_4, M_PI/6}, float delta_cur=1.);
template<typename T>
std::vector<float> linspace(T start_in, T end_in, int num_in);

float aStar(Node2D& start, Node2D& goal, Node2D* nodes2D, int width, int height, CollisionDetection& configurationSpace, Visualize& visualization);

bool map_isok_DSU(Node3D& start, const Node3D& goal, Node3D* nodes3D, int width, int height,bool,CollisionDetection& configurationSpace,
                   std::vector<int>* externalParent = nullptr, std::vector<int>* externalRank = nullptr, bool* externalInitialized = nullptr);

bool map_isok_Astar(Node3D& start, const Node3D& goal, Node3D* nodes3D, int width, int height,bool,CollisionDetection& configurationSpace);
Node3D* aStar3D(Node3D& start, const Node3D& goal, Node3D* nodes3D, int width, int height,bool,Node3D* lookup_traversable);
Node3D* aStar3D_H(Node3D& start, const Node3D& goal, Node3D* nodes3D, int width, int height,bool

  ,CollisionDetection& configurationSpace,bool *way_finded, int *planning_iterations_output = nullptr, double *path_cost_output = nullptr);

void updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D, float* dubinsLookup, int width, int height, CollisionDetection& configurationSpace, Visualize& visualization);
void update3Dmap(Node3D* nodes3D, int width, int height, CollisionDetection& configurationSpace);
Node3D* dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace);
void initPathPublisher(ros::NodeHandle& nh);
void show_const_sampling(const std::vector<Eigen::Vector3d>& sampling_points,const Color color,const std::string& ns);
float calculateHeuristic_M(Node3D& current,const Node3D& goal,

  int width,int height) ;
double omni_w_max,side_v;
bool caculate_time = true;
ros::Time t0 ;
ros::Time t1 ;

struct CompareNodes {

  bool operator()(const Node3D* lhs, const Node3D* rhs) const {
    return lhs->getC() > rhs->getC();
  }

  bool operator()(const Node2D* lhs, const Node2D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
};

bool isFloatEqual(double a, double b, double epsilon = 1e-9) {
    return std::abs(a - b) < epsilon;
}

void showpath(Node3D* endnode,int width,int height,const std::string& ns,const Color color)
{
  clearMarkers("");
  clearMarkers_openset("");
  int totalNodes = width * height * Constants::positions * Constants::headings;
  const Node3D* pathnode = endnode;
  int id=0;
  while (pathnode->getPred() !=nullptr){

    publishMarker(*pathnode,color,id,ns,0.1);
    pathnode=pathnode->getPred();
    id++;
  }
}

Node3D* Algorithm::hybridAStar(Node3D& start,
                               const Node3D& goal,
                               Node3D* nodes3D,
                               Node2D* nodes2D,
                               int width,
                               int height,
                               CollisionDetection& configurationSpace,
                               float* dubinsLookup,
                               Visualize& visualization) {

  int iPred, iSucc;
  float newG;

  int dir = Constants::reverse ? 6 : 3;

  int iterations = 0;

  ros::Duration d(0.001);

  typedef boost::heap::binomial_heap<Node3D*,
          boost::heap::compare<CompareNodes>
          > priorityQueue;
  priorityQueue O;

  updateH(start, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization);

  start.open();

  O.push(&start);
  iPred = start.setIdx(width, height);
  nodes3D[iPred] = start;

  Node3D* nPred;
  Node3D* nSucc;

  while (!O.empty()) {

    nPred = O.top();

    iPred = nPred->setIdx(width, height);
    iterations++;

    if (Constants::visualization) {
      visualization.publishNode3DPoses(*nPred);
      visualization.publishNode3DPose(*nPred);
      d.sleep();
    }

    if (nodes3D[iPred].isClosed()) {

      O.pop();
      continue;
    }

    else if (nodes3D[iPred].isOpen()) {

      nodes3D[iPred].close();

      O.pop();

      if (*nPred == goal || iterations > Constants::iterations) {

        return nPred;
      }

      else {

        if (Constants::dubinsShot && nPred->isInRange(goal) && nPred->getPrim() < 3) {
          nSucc = dubinsShot(*nPred, goal, configurationSpace);

          if (nSucc != nullptr && *nSucc == goal) {

            return nSucc;
          }
        }

        for (int i = 0; i < dir; i++) {

          nSucc = nPred->createSuccessor(i);

          iSucc = nSucc->setIdx(width, height);

          if (nSucc->isOnGrid(width, height) && configurationSpace.isTraversable(nSucc)) {

            if (!nodes3D[iSucc].isClosed() || iPred == iSucc) {

              nSucc->updateG();
              newG = nSucc->getG();

              if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG() || iPred == iSucc) {

                updateH(*nSucc, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization);

                if (iPred == iSucc && nSucc->getC() > nPred->getC() + Constants::tieBreaker) {
                  delete nSucc;
                  continue;
                }

                else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + Constants::tieBreaker) {
                  nSucc->setPred(nPred->getPred());
                }

                if (nSucc->getPred() == nSucc) {
                  std::cout << "looping";
                }

                nSucc->open();
                nodes3D[iSucc] = *nSucc;
                O.push(&nodes3D[iSucc]);
                delete nSucc;
              } else { delete nSucc; }
            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }

  if (O.empty()) {
    return nullptr;
  }

  return nullptr;
}

Node3D* Algorithm::hybridAStar_M(Node3D& start,
                               const Node3D& goal,
                               Node3D* nodes3D,
                               int width,
                               int height,
                               CollisionDetection& configurationSpace,
                               float* dubinsLookup,

                               std::vector<int>* dsuParent,
                               std::vector<int>* dsuRank,
                               bool* dsuInitialized,

                               double* t_judge_output_Astar,
                               double* t_judge_output_DSU,
                               double* t_planning_output,

                               bool *path_is_feasible_Astar,
                               bool *path_is_feasible_DSU,

                               bool *path_is_feasible_searching,
                               int *planning_iterations_output,
                               double *path_cost_output
                               ) {

  printf("使用的算法框架：我的算法框架\r\n");

  int iPred, iSucc;
  float newG;

  int dir = Constants::reverse ? 6 : 3;

  int iterations = 0;

  ros::Duration d(0.001);

  typedef boost::heap::binomial_heap<Node3D*,
          boost::heap::compare<CompareNodes>
          > priorityQueue;
  priorityQueue O;

  ros::Time t11=ros::Time::now();

  static bool initialized = false;
  if (!initialized) {
    static ros::NodeHandle nh;
    initPathPublisher(nh);
    initialized = true;
  }

  if((!configurationSpace.configurationTest_polygon(start.getX(),start.getY(),start.getT()))
      || (!configurationSpace.configurationTest_polygon(goal.getX(),goal.getY(),goal.getT())) ){

    printf("起点或终点不可通行!  \r\n");
    if (path_is_feasible_searching != nullptr) {
      *path_is_feasible_searching = false;
    }
    if (t_planning_output != nullptr) {
      *t_planning_output = 0.0;
    }
    if (planning_iterations_output != nullptr) {
      *planning_iterations_output = 0;
    }
    if (path_cost_output != nullptr) {
      *path_cost_output = 0.0;
    }
    return nullptr;
  }else{
    printf("起点和终点可通行!  \r\n");

  }
  ros::Time t12=ros::Time::now();
  ros::Duration d11(t12 - t11);
  std::cout << "判断起点可行程序 in ms: " << d11 * 1000 << std::endl;

  if(!HybridAStar::Constants::output_samples){

      printf("\r\n");
    std::cout << " 开始使用A*判断道路是否通行" << std::endl;
    ros::Time t15_astar_judge=ros::Time::now();
    if(!map_isok_Astar(start,goal,nodes3D,width,height,false,configurationSpace) ){
      std::cout << "初步判定：不可通行!"<< " (A*) " << std::endl;
      *path_is_feasible_Astar=false;

    }else {
      *path_is_feasible_Astar=true;
      std::cout << "初步判定：可通行!"<< " (A*) " << std::endl;

    }
      ros::Time t16_astar_judge=ros::Time::now();
      ros::Duration t_astar_judge(t16_astar_judge - t15_astar_judge);
      std::cout << "用时 in ms: " << " (A*) "<< t_astar_judge * 1000 << std::endl;

      if (t_judge_output_Astar != nullptr) {
        *t_judge_output_Astar = t_astar_judge.toSec()*1000;

      }
    printf("\r\n");

    std::cout << " 开始使用DSU判断道路是否通行" << std::endl;
    ros::Time t15_dsu_judge=ros::Time::now();
    if(!map_isok_DSU(start,goal,nodes3D,width,height,false,configurationSpace,
                    dsuParent, dsuRank, dsuInitialized) ){

      std::cout << "初步判定：不可通行!" << " (DSU) "<< std::endl;
      *path_is_feasible_DSU=false;

    }else {
          *path_is_feasible_DSU=true;
      std::cout << "初步判定：可通行!"<< " (DSU) " << std::endl;

    }
      ros::Time t16_dsu_judge=ros::Time::now();
      ros::Duration t_dsu_judge(t16_dsu_judge - t15_dsu_judge);
      std::cout << "用时 in ms: "<< " (DSU) " << t_dsu_judge * 1000 << std::endl;

    if (t_judge_output_DSU != nullptr) {
      *t_judge_output_DSU = t_dsu_judge.toSec()*1000;

    }
  }

  ros::Time t_searching_begin = ros::Time::now();

  bool way_finded=false;

  printf("\r\n");
  Node3D* result = aStar3D_H(start,goal,nodes3D, width, height,false,configurationSpace,&way_finded, planning_iterations_output, path_cost_output);
  printf("\r\n");

  if (path_is_feasible_searching != nullptr) {
    *path_is_feasible_searching = way_finded;
  }

  ros::Time t_searching_end = ros::Time::now();
  ros::Duration t_searching(t_searching_end-t_searching_begin);
  if (t_planning_output != nullptr) {
    *t_planning_output = t_searching.toSec()*1000;
  }

  return result;
}

float aStar(Node2D& start,
            Node2D& goal,
            Node2D* nodes2D,
            int width,
            int height,
            CollisionDetection& configurationSpace,
            Visualize& visualization) {

  int iPred, iSucc;
  float newG;

  for (int i = 0; i < width * height; ++i) {
    nodes2D[i].reset();
  }

  ros::Duration d(0.001);

  boost::heap::binomial_heap<Node2D*,
        boost::heap::compare<CompareNodes>> O;

  start.updateH(goal);

  start.open();

  O.push(&start);
  iPred = start.setIdx(width);
  nodes2D[iPred] = start;

  Node2D* nPred;
  Node2D* nSucc;

  while (!O.empty()) {

    nPred = O.top();

    iPred = nPred->setIdx(width);

    if (nodes2D[iPred].isClosed()) {

      O.pop();
      continue;
    }

    else if (nodes2D[iPred].isOpen()) {

      nodes2D[iPred].close();
      nodes2D[iPred].discover();

      if (Constants::visualization2D) {
        visualization.publishNode2DPoses(*nPred);
        visualization.publishNode2DPose(*nPred);
        d.sleep();
      }

      O.pop();

      if (*nPred == goal) {
        return nPred->getG();
      }

      else {

        for (int i = 0; i < Node2D::dir; i++) {

          nSucc = nPred->createSuccessor(i);

          iSucc = nSucc->setIdx(width);

          if (nSucc->isOnGrid(width, height) &&  configurationSpace.isTraversable(nSucc) && !nodes2D[iSucc].isClosed()) {

            nSucc->updateG();
            newG = nSucc->getG();

            if (!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].getG()) {

              nSucc->updateH(goal);

              nSucc->open();
              nodes2D[iSucc] = *nSucc;
              O.push(&nodes2D[iSucc]);
              delete nSucc;
            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }

  return 1000;
}

void update3Dmap(Node3D* nodes3D, int width, int height, CollisionDetection& configurationSpace)
{

    const int positionResolution = Constants::positionResolution;
    const int headings = Constants::headings;
    const float deltaHeadingRad = Constants::deltaHeadingRad;

    int totalConfigurations = width * height * positionResolution * positionResolution * headings;

    std::cout << "Updating 3D map with " << totalConfigurations << " configurations..." << std::endl;

    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            for (int iX = 0; iX < positionResolution; ++iX) {
                for (int iY = 0; iY < positionResolution; ++iY) {
                    for (int iT = 0; iT < headings; ++iT) {

                        float realX = x + (float)iX / positionResolution;
                        float realY = y + (float)iY / positionResolution;
                        float realT = iT * deltaHeadingRad;

                        int nodeIndex = y * width * positionResolution * positionResolution * headings +
                                       x * positionResolution * positionResolution * headings +
                                       iY * positionResolution * headings +
                                       iX * headings +
                                       iT;

                        if (nodeIndex < 0 || nodeIndex >= totalConfigurations) {
                            std::cerr << "Error: Invalid node index " << nodeIndex << std::endl;
                            continue;
                        }

                        bool isCollisionFree = configurationSpace.configurationTest_polygon(realX, realY, realT);

                        if(nodeIndex==21960){

                        }

                        nodes3D[nodeIndex] = Node3D(realX, realY, realT, 0.0f, 0.0f, nullptr);

                        if (isCollisionFree) {

                            nodes3D[nodeIndex].setIsOk(true);
                        } else {

                            nodes3D[nodeIndex].setIsOk(false);
                        }
                    }
                }
            }
        }

        if (width > 10 && x % (width / 10) == 0) {
            std::cout << "Progress: " << (x * 100 / width) << "%" << std::endl;
        }
    }

    std::cout << "3D map update completed!" << std::endl;
}

void  updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D, float* dubinsLookup, int width, int height, CollisionDetection& configurationSpace, Visualize& visualization) {
  float dubinsCost = 0;
  float reedsSheppCost = 0;
  float twoDCost = 0;
  float twoDoffset = 0;
  float eulaDis = 0;

  if (Constants_aws::eula_dis){
    eulaDis = sqrt((start.getX() - goal.getX()) * (start.getX() - goal.getX())
              + (start.getY() - goal.getY()) * (start.getY() - goal.getY()));
    if (Constants_aws::time_measurement) eulaDis = eulaDis/Constants_aws::max_wheel_v;
    twoDCost = eulaDis;
  }

  if (Constants_aws::heading_heuristic) {
    float cost_yaw = std::abs(
      Helper::normalizeHeadingRad(start.getT() - goal.getT()));
    if(Constants_aws::time_measurement) cost_yaw =
              cost_yaw/Constants_aws::max_steering_v;

    twoDCost = 1.3*std::max((float)1.3*twoDCost , cost_yaw);

  }

  start.setH(std::max(reedsSheppCost, std::max(dubinsCost, twoDCost)));
}

Node3D* dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace) {

  double q0[] = { start.getX(), start.getY(), start.getT() };

  double q1[] = { goal.getX(), goal.getY(), goal.getT() };

  DubinsPath path;

  dubins_init(q0, q1, Constants::r, &path);

  int i = 0;
  float x = 0.f;
  float length = dubins_path_length(&path);

  static std::vector<std::unique_ptr<Node3D[]>> dubinsPaths;
  size_t nodeCount = static_cast<size_t>(length / Constants::dubinsStepSize) + 1;

  auto pathNodes = std::make_unique<Node3D[]>(nodeCount);
  dubinsPaths.push_back(std::move(pathNodes));
  Node3D* dubinsNodes = dubinsPaths.back().get();

  x += Constants::dubinsStepSize;
  while (x <  length) {
    double q[3];
    dubins_path_sample(&path, x, q);
    dubinsNodes[i].setX(q[0]);
    dubinsNodes[i].setY(q[1]);
    dubinsNodes[i].setT(Helper::normalizeHeadingRad(q[2]));

    if (configurationSpace.isTraversable(&dubinsNodes[i])) {

      if (i > 0) {
        dubinsNodes[i].setPred(&dubinsNodes[i - 1]);
      } else {
        dubinsNodes[i].setPred(&start);
      }

      if (&dubinsNodes[i] == dubinsNodes[i].getPred()) {
        std::cout << "looping shot";
      }

      x += Constants::dubinsStepSize;
      i++;
    } else {

      if (!dubinsPaths.empty()) {
        dubinsPaths.pop_back();
      }
      return nullptr;
    }
  }

  return &dubinsNodes[i - 1];
}

void resetNode(Node3D& node) {
    node.setG(0);
    node.setH(0);
    node.setPred(nullptr);
    if (node.isOpen() || node.isClosed()) {

    }
}

float normalizeAngle(float angle) {
    while (angle < 0) angle += 2.0f * M_PI;
    while (angle >= 2.0f * M_PI) angle -= 2.0f * M_PI;
    return angle;
}

float calculateHeuristic(const Node3D& current, const Node3D& goal) {

    double change_T=HybridAStar::Constants::V_max_vision/HybridAStar::Constants::adec
              +HybridAStar::Constants::V_max_vision/HybridAStar::Constants::amax;
    int tip_last=current.get_index_neighbor_simple();

    float dx = goal.getX() - current.getX();
    float dy = goal.getY() - current.getY();
    int integrate_dx=(int(dx)==0)&&(dx-int(dx)<=1.0f/Constants::positionResolution);
    int integrate_dy=(int(dy)==0)&&(dy-int(dy)<=1.0f/Constants::positionResolution);
    bool need_change=(integrate_dx!=0)&&(integrate_dy!=0);
    int need_tip=-1;
    if(need_change==0){
      if(integrate_dx==0 && integrate_dy!=0){
        if(dy>0){need_tip=4;}
        if(dy<0){need_tip=5;}
      }else if(integrate_dy==0 && integrate_dx!=0){
        if(dx>0){need_tip=2;}
        if(dx<0){need_tip=3;}
      }
    }
    if(need_tip==tip_last&&need_change==0){
      need_change=0;
    }else{
      need_change=1;
    }
    float Distance1 = std::sqrt(dx*dx + dy*dy)/side_v;
    float Distance2 = (std::abs(dx)+ std::abs(dy))/side_v;
    float Distance3 = Distance2 + need_change*change_T;

    float angleDiff = std::abs(normalizeAngle(goal.getT() - current.getT()));
    float shortestAngleDiff = std::min(angleDiff, (float)(2*M_PI - angleDiff))/omni_w_max;

    float spatialWeight = 1.0f;
    float angularWeight = 1.0f;

    float cost = Distance1 + shortestAngleDiff;
    return cost ;
}

float CalculateHeuristic_Simplist_TriD(const Node3D& current, const Node3D& goal) {

    float dx = goal.getX() - current.getX();
    float dy = goal.getY() - current.getY();
    float dT = std::abs(normalizeAngle(goal.getT() - current.getT()));
    float Distance = std::abs(dx)+ std::abs(dy)+std::abs(dT);
    float ratio = 2.0;
    return ratio*Distance ;
}

float calculateHeuristic_M(Node3D& current, const Node3D& goal,

  int width,int height) {

  Node3D goal_for_caculate_h = goal;
  Node3D current_for_caculate_h = current;
  float h1=0;

  float dt = std::abs(current.getT()-goal.getT());
  float dx = std::abs(current.getX()-goal.getX());
  float dy = std::abs(current.getY()-goal.getY());
  double V_max = HybridAStar::Constants::V_max_vision;
  double amax = HybridAStar::Constants::amax;
  double adec = HybridAStar::Constants::adec;
  double w_max = HybridAStar::Constants::w_max_wheel;
  if(Constants::fix_axil){

    return dt/w_max+(std::sqrt(dx*dx+dy*dy))/V_max;
  }
  else{
    float t_omni = ((dt < M_PI) ? dt : (M_PI*2.0f-dt))/omni_w_max;
    float t_side = sqrt((current.getX()-goal.getX())*(current.getX()-goal.getX())+
                  (current.getY()-goal.getY())*(current.getY()-goal.getY()))/side_v;
    float h2=t_omni+t_side;

    return max(h1,h2);
  }
}

bool isGoalReached(const Node3D& current, const Node3D& goal) {
  if(current==goal){return true;}
  return false;
}

bool isValidConfiguration(float x, float y, float t, int width, int height) {
    return (x >= 0 && x < width && y >= 0 && y < height && t >= 0 && t < 2*M_PI);
}

struct NeighborInfo {
    float x, y, t;
    float cost;
    int tip;
    NeighborInfo(float x_, float y_, float t_, float cost_,float tip_)
        : x(x_), y(y_), t(t_), cost(cost_), tip(tip_) {}
};

std::vector<NeighborInfo> generateNeighbors(const Node3D& current) {
    std::vector<NeighborInfo> neighbors;

    const float posStep_x = 1.0f/Constants::positionResolution;
    const float posStep_y = 1.0f/Constants::positionResolution;
    const float posStep_z = Constants::deltaHeadingRad;

    double v=HybridAStar::Constants::V_max_vision;

    neighbors.emplace_back(current.getX()+posStep_x,
                        current.getY()+posStep_y,
                        current.getT(),
                        std::sqrt(posStep_x*posStep_x+posStep_y*posStep_y)/v
                        ,
                      0);
    neighbors.emplace_back(current.getX()-posStep_x,
                        current.getY()-posStep_y,
                        current.getT(),
                        std::sqrt(posStep_x*posStep_x+posStep_y*posStep_y)/v
                        ,
                      1);
    neighbors.emplace_back(current.getX()+posStep_x,
                        current.getY()-posStep_y,
                        current.getT(),
                        std::sqrt(posStep_x*posStep_x+posStep_y*posStep_y)/v
                        ,
                      2);
    neighbors.emplace_back(current.getX()-posStep_x,
                        current.getY()+posStep_y,
                        current.getT(),
                        std::sqrt(posStep_x*posStep_x+posStep_y*posStep_y)/v
                        ,
                      3);
    neighbors.emplace_back(current.getX()+posStep_x,
                        current.getY(),
                        current.getT(),
                        std::sqrt(posStep_x)/v
                        ,
                      4);
    neighbors.emplace_back(current.getX(),
                        current.getY()+posStep_y,
                        current.getT(),
                        std::sqrt(posStep_y)/v
                        ,
                      5);
    neighbors.emplace_back(current.getX()-posStep_x,
                        current.getY(),
                        current.getT(),
                        std::sqrt(posStep_x)/v
                        ,
                      6);
    neighbors.emplace_back(current.getX(),
                        current.getY()-posStep_y,
                        current.getT(),
                        std::sqrt(posStep_y)/v
                        ,
                      7);
    return neighbors;

}

std::vector<NeighborInfo> GenerateNeighbors_Simplist_TriD(const Node3D& current) {
    std::vector<NeighborInfo> neighbors;

    const float posStep_x =  1.0f / Constants::positionResolution;
    const float posStep_y =  1.0f / Constants::positionResolution;
    const float posStep_z =  Constants::deltaHeadingRad;

    double v = HybridAStar::Constants::V_max_vision;
    double w = omni_w_max;

    neighbors.emplace_back(current.getX(),
                           current.getY() ,
                           normalizeAngle(current.getT() + posStep_z),
                           posStep_z,
                           0);
    neighbors.emplace_back(current.getX(),
                           current.getY() ,
                           normalizeAngle(current.getT() - posStep_z),
                           posStep_z,
                           1);

    neighbors.emplace_back(current.getX() + posStep_x,
                           current.getY() ,
                           current.getT(),
                           posStep_x,
                           2);
    neighbors.emplace_back(current.getX() - posStep_x,
                           current.getY() ,
                           current.getT(),
                           posStep_x,
                           3);

    neighbors.emplace_back(current.getX(),
                           current.getY() + posStep_y ,
                           current.getT(),
                           posStep_y,
                           4);
    neighbors.emplace_back(current.getX(),
                           current.getY() - posStep_y,
                           current.getT(),
                           posStep_y,
                           5);

    return neighbors;
}

int tippp=0;

Node3D* aStar3D(Node3D& start, const Node3D& goal, Node3D* nodes3D, int width, int height,bool showflag,Node3D* lookup_traversable) {
    int id = 0;
    tippp++;
    clearMarkers("");
    clearMarkers_openset("");

    if(tippp==276){

    }

    std::cout << "Starting 3D A* search..." << std::endl;
    std::cout << "Start: (" << start.getX() << ", " << start.getY()
              << ", " << start.getT() << ")" << std::endl;
    std::cout << "End:  (" << goal.getX() << ", " << goal.getY()
              << ", " << goal.getT() << ")" << std::endl;

    SamplingResult sample_table = SamplingResult::createWithDefaults();
    omni_w_max=sample_table.getomni_w_max();
    side_v=HybridAStar::Constants::V_max_vision;

    int totalNodes = width * height * Constants::positions * Constants::headings;

    for (int i = 0; i < totalNodes; i++) {
        nodes3D[i].reset();
        nodes3D[i].setG(0.0f);
        nodes3D[i].setH(0.0f);
        nodes3D[i].setPred(nullptr);

    }

    typedef boost::heap::binomial_heap<Node3D*,
          boost::heap::compare<CompareNodes>
          > priorityQueue;
    priorityQueue openSet;
    std::unordered_set<int> openSetIndices;

    int startIdx = start.setIdxM(start.getX(),start.getY(),start.getT(),width,height);

    if (startIdx < 0 || startIdx >= totalNodes) {
        std::cout << "Invalid start index: " << startIdx << std::endl;
        return nullptr;
    }

    nodes3D[startIdx] = start;
    nodes3D[startIdx].setG(0.0f);
    nodes3D[startIdx].setH(calculateHeuristic(start, goal));
    nodes3D[startIdx].open();

    openSet.push(&nodes3D[startIdx]);
    openSetIndices.insert(startIdx);

    int iterations = 0;
    int nodesExpanded = 0;

    while (!openSet.empty() && iterations < Constants::iterations*Constants::positions) {
        iterations++;
        if(tippp==1&&iterations==15){

        }

        Node3D* current = openSet.top();
        openSet.pop();

        int currentIdx = current->setIdxM(current->getX(),current->getY(),current->getT(),width,height);
        openSetIndices.erase(currentIdx);

        if (current->isClosed()) {
            continue;
        }

        current->close();
        nodesExpanded++;

        if (current != nullptr) {

            publishMarker_openset(*current,GREEN,id,"local_good_node",0.3);
            publishMarker(*current,WHITE,id,"local_good_node",0.4);
            printf("current : (%.4f , %.4f, %.4f)\r\n",current->getX(),current->getY(),current->getT());
            id++;
        }

        if (isGoalReached(*current, goal)) {
            std::cout << "Goal reached!" << std::endl;
            std::cout << "Iterations: " << iterations << std::endl;
            std::cout << "Nodes expanded: " << nodesExpanded << std::endl;
            std::cout << "Path cost: " << current->getG() << std::endl;
            showpath(current, width, height,"finalpath",GREEN);
            plt_M.publishPath(current);
            return current;
        }

        std::vector<NeighborInfo> neighbors = generateNeighbors(*current);

        clearMarkers("local_good_node");
        for (const auto& neighborInfo : neighbors) {

            if (!isValidConfiguration(neighborInfo.x, neighborInfo.y, neighborInfo.t,
                                    width, height)) {
                continue;
            }

            Node3D tempNeighbor(neighborInfo.x, neighborInfo.y, neighborInfo.t, 0, 0, nullptr);

            int neighborIdx = tempNeighbor.setIdxM(tempNeighbor.getX(),tempNeighbor.getY(),tempNeighbor.getT(),width,height);

              publishMarker_openset(tempNeighbor,BLUE,id,"neighbor_points",0.2);

               id++;

            if (neighborIdx < 0 || neighborIdx >= totalNodes) {
                continue;
            }

            if (!lookup_traversable[neighborIdx].getIsOk()) {
                publishMarker(tempNeighbor,RED,id,"obs_points",0.5);
                id++;
                continue;
            }

            if (nodes3D[neighborIdx].isClosed()) {
                continue;
            }

            float newG = current->getG() + neighborInfo.cost;

            bool inOpenSet = openSetIndices.count(neighborIdx) > 0;

            if (!inOpenSet || newG < nodes3D[neighborIdx].getG()) {
                nodes3D[neighborIdx].setX(neighborInfo.x);
                nodes3D[neighborIdx].setY(neighborInfo.y);
                nodes3D[neighborIdx].setT(neighborInfo.t);
                nodes3D[neighborIdx].setG(newG);
                nodes3D[neighborIdx].setH(calculateHeuristic(nodes3D[neighborIdx], goal));
                nodes3D[neighborIdx].setPred(current);
                nodes3D[neighborIdx].set_index_neighbor_simple(neighborInfo.tip);
                if (!inOpenSet) {
                    nodes3D[neighborIdx].open();
                    openSet.push(&nodes3D[neighborIdx]);
                    openSetIndices.insert(neighborIdx);
                }
            }
        }

        if (iterations % 5000 == 0) {
            std::cout << "Progress - Iterations: " << iterations
                      << ", Expanded: " << nodesExpanded
                      << ", Open set size: " << openSet.size() << std::endl;
        }
    }

    std::cout << "No path found!" << std::endl;
    std::cout << "Total iterations: " << iterations << std::endl;
    std::cout << "Nodes expanded: " << nodesExpanded << std::endl;
    return nullptr;
}

bool map_isok_Astar(Node3D& start, const Node3D& goal, Node3D* nodes3D, int width, int height,bool showflag,CollisionDetection& configurationSpace) {
    int id = 0;
    tippp++;
    if(showflag){
      clearMarkers("");
      clearMarkers_openset("");
    }
    printf("开始判断道路可通行能力");
    if(tippp==276){

    }

    std::cout << "Starting 3D A* search..." << std::endl;
    std::cout << "Start: (" << start.getX() << ", " << start.getY()
              << ", " << start.getT() << ")" << std::endl;
    std::cout << "End:  (" << goal.getX() << ", " << goal.getY()
              << ", " << goal.getT() << ")" << std::endl;

    int totalNodes = width * height * Constants::positions * Constants::headings;

    for (int i = 0; i < totalNodes; i++) {
        nodes3D[i].reset();
        nodes3D[i].setG(0.0f);
        nodes3D[i].setH(0.0f);
        nodes3D[i].setPred(nullptr);

    }

    typedef boost::heap::binomial_heap<Node3D*,
          boost::heap::compare<CompareNodes>
          > priorityQueue;
    priorityQueue openSet;
    std::unordered_set<int> openSetIndices;

    int startIdx = start.setIdxM(start.getX(),start.getY(),start.getT(),width,height);

    if (startIdx < 0 || startIdx >= totalNodes) {
        std::cout << "Invalid start index: " << startIdx << std::endl;
        return false;
    }

    nodes3D[startIdx] = start;
    nodes3D[startIdx].setG(0.0f);
    nodes3D[startIdx].setH(CalculateHeuristic_Simplist_TriD(start, goal));
    nodes3D[startIdx].open();

    openSet.push(&nodes3D[startIdx]);
    openSetIndices.insert(startIdx);

    int iterations = 0;
    int nodesExpanded = 0;

    while (!openSet.empty() && iterations < Constants::iterations*Constants::positions) {
        iterations++;
        if(tippp==1 && iterations==15){

        }

        Node3D* current = openSet.top();
        openSet.pop();

        int currentIdx = current->setIdxM(current->getX(),current->getY(),current->getT(),width,height);
        openSetIndices.erase(currentIdx);

        if (current->isClosed()) {
            continue;
        }

        current->close();
        nodesExpanded++;

        if (current != nullptr&&showflag) {
            publishMarker_openset(*current,GREEN,id,"local_good_node",0.3);
            publishMarker(*current,WHITE,id,"local_good_node",0.4);

            id++;
        }

        if (isGoalReached(*current, goal)) {
            std::cout << "Goal reached!" << std::endl;
            std::cout << "Iterations: " << iterations << std::endl;
            std::cout << "Nodes expanded: " << nodesExpanded << std::endl;
            std::cout << "Path cost: " << current->getG() << std::endl;

            return true;
        }

        std::vector<NeighborInfo> neighbors = GenerateNeighbors_Simplist_TriD(*current);

        if(showflag){clearMarkers("local_good_node");}
        for (const auto& neighborInfo : neighbors) {

            if (!isValidConfiguration(neighborInfo.x, neighborInfo.y, neighborInfo.t,
                                    width, height)) {
                continue;
            }

            Node3D tempNeighbor(neighborInfo.x, neighborInfo.y, neighborInfo.t, 0, 0, nullptr);

            int neighborIdx = tempNeighbor.setIdxM(tempNeighbor.getX(),tempNeighbor.getY(),tempNeighbor.getT(),width,height);

            if(showflag){

               id++;
            }
            if (neighborIdx < 0 || neighborIdx >= totalNodes) {
                continue;
            }

            if (!configurationSpace.isTraversable(&tempNeighbor)) {
              if(showflag){publishMarker(tempNeighbor,RED,id,"obs_points",0.5);}

                id++;
                continue;
            }

            if (nodes3D[neighborIdx].isClosed()) {
                continue;
            }

            float newG = current->getG() + neighborInfo.cost;

            bool inOpenSet = openSetIndices.count(neighborIdx) > 0;

            if (!inOpenSet || newG < nodes3D[neighborIdx].getG()) {
                nodes3D[neighborIdx].setX(neighborInfo.x);
                nodes3D[neighborIdx].setY(neighborInfo.y);
                nodes3D[neighborIdx].setT(neighborInfo.t);
                nodes3D[neighborIdx].setG(newG);
                nodes3D[neighborIdx].setH(CalculateHeuristic_Simplist_TriD(nodes3D[neighborIdx], goal));
                nodes3D[neighborIdx].setPred(current);

                if (!inOpenSet) {
                    nodes3D[neighborIdx].open();
                    openSet.push(&nodes3D[neighborIdx]);
                    openSetIndices.insert(neighborIdx);
                }
            }
        }

        if (iterations % 5000 == 0) {
            std::cout << "Progress - Iterations: " << iterations
                      << ", Expanded: " << nodesExpanded
                      << ", Open set size: " << openSet.size() << std::endl;
        }
    }

    std::cout << "No path found!" << std::endl;
    std::cout << "Total iterations: " << iterations << std::endl;
    std::cout << "Nodes expanded: " << nodesExpanded << std::endl;
    return false;
}

bool map_isok_DSU(Node3D& start, const Node3D& goal, Node3D* nodes3D, int width, int height,bool showflag,CollisionDetection& configurationSpace,
                  std::vector<int>* externalParent, std::vector<int>* externalRank, bool* externalInitialized){

    int totalNodes = width*Constants::positionResolution * height*Constants::positionResolution * Constants::headings;

    bool External_exist = (externalParent != nullptr && externalRank != nullptr && externalInitialized != nullptr &&
    externalParent->size() == static_cast<size_t>(totalNodes) );
    printf("开始用并查集判断道路可通行能力\n");

    if (!External_exist) {
          externalParent->resize(totalNodes, -1);
          externalRank->resize(totalNodes, 0);
        } else {
            printf("使用已经构建过的并查集\n");
        }

    std::vector<int>& parent = *externalParent;
    std::vector<int>& rank =  *externalRank ;

    std::function<int(int)> find = [&](int x) {
        if (parent[x] != x) {
            parent[x] = find(parent[x]);
        }
        return parent[x];
    };

    auto unite = [&](int x, int y) {
        int px = find(x);
        int py = find(y);
        if (px == py) return;

        if (rank[px] < rank[py]) {
            parent[px] = py;
        } else if (rank[px] > rank[py]) {
            parent[py] = px;
        } else {
            parent[py] = px;
            rank[px]++;
        }
    };

    if (!External_exist) {
        ros::Time t_build_start = ros::Time::now();
        printf("开始构建并查集...\n");

        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                for (int ix = 0; ix < Constants::positionResolution; ix++) {
                  for (int iy = 0; iy < Constants::positionResolution; iy++) {
                    for (int it = 0; it < Constants::headings; it++) {
                        int index_m = y * width * Constants::positionResolution * Constants::positionResolution * Constants::headings +
                            x * Constants::positionResolution * Constants::positionResolution * Constants::headings +
                            iy * Constants::positionResolution * Constants::headings +
                            ix * Constants::headings +
                            it;
                        float x_real=x+ix*Constants::cellSize/Constants::positionResolution;
                        float y_real=y+iy*Constants::cellSize/Constants::positionResolution;
                        float t_real=it*Constants::deltaHeadingRad;

                        Node3D* current = new Node3D(x_real, y_real, t_real, 0, 0, nullptr, 0);

                        if ( !configurationSpace.configurationTest_polygon(x_real,y_real,t_real)) {
                            delete current;
                            continue;
                        }

                        if (parent[index_m] == -1) {
                            parent[index_m] = index_m;
                        }

                        for (int i = 0; i < Node3D::dir*2; i++) {

                            Node3D* nSucc = current->createSuccessor_DSU(i);
                            int succIdx = nSucc->setIdxM(nSucc->getX(),nSucc->getY(),nSucc->getT(),width,height);

                            if (nSucc->isOnGrid(width, height) && configurationSpace.configurationTest_polygon(nSucc->getX(),nSucc->getY(),nSucc->getT())) {
                                if (parent[succIdx] == -1) {
                                    parent[succIdx] = succIdx;
                                }

                                unite(index_m, succIdx);
                            }

                            delete nSucc;
                        }
                        delete current;
                      }
                  }
                }
            }
        }
        ros::Duration d_build = ros::Time::now() - t_build_start;
        printf("并查集构建完成，耗时: %.2f ms\n", d_build.toSec() * 1000.0);

        *externalInitialized = true;
        printf("外部并查集已缓存\n");

    }

    Node3D startCopy = start;
    Node3D goalCopy = goal;
    int startIdx = startCopy.setIdxM(start.getX(),start.getY(),start.getT(),width,height);
    int goalIdx = goalCopy.setIdxM(goal.getX(),goal.getY(),goal.getT(),width,height);

    if(showflag){

        auto addPointToSet = [](std::vector<geometry_msgs::Point>& points, float x, float y, float z) {
            geometry_msgs::Point point;
            point.x = x;
            point.y = y;
            point.z = z;
            points.push_back(point);
        };

        std::vector<geometry_msgs::Point> startPoints;
        std::vector<geometry_msgs::Point> goalPoints;

        int ix = 0;
        int iy = 0;
        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
            for (int ix = 0; ix < Constants::positionResolution; ix++) {
              for (int iy = 0; iy < Constants::positionResolution; iy++) {
                for (int it = 0; it < Constants::headings; it++) {

                    int index_m = y * width * Constants::positionResolution * Constants::positionResolution * Constants::headings +
                        x * Constants::positionResolution * Constants::positionResolution * Constants::headings +
                        iy * Constants::positionResolution * Constants::headings +
                        ix * Constants::headings +
                        it;

                    float px = x + ix * Constants::cellSize / Constants::positionResolution;
                    float py = y + iy * Constants::cellSize / Constants::positionResolution;
                    float pz = 0;

                    int rootM = (parent[index_m] != -1) ? find(index_m) : -1;
                    int rootStart = (parent[startIdx] != -1) ? find(startIdx) : -1;
                    int rootGoal = (parent[goalIdx] != -1) ? find(goalIdx) : -1;

                    if(rootM == rootStart && rootM != -1){

                        addPointToSet(startPoints, px, py, pz);
                    }
                    else if(rootM == rootGoal && rootM != -1){

                        addPointToSet(goalPoints, px, py, pz);
                    }
                }
            }
        }
      }
    }

        if (!startPoints.empty()) {
            visualization_msgs::Marker startMarker;
            startMarker.header.frame_id = "map";
            startMarker.header.stamp = ros::Time::now();
            startMarker.ns = "start_same_node";
            startMarker.id = 0;
            startMarker.type = visualization_msgs::Marker::POINTS;
            startMarker.action = visualization_msgs::Marker::ADD;
            startMarker.points = startPoints;
            startMarker.scale.x = 0.3;
            startMarker.scale.y = 0.3;
            startMarker.scale.z = 0.0;
            startMarker.color.r = GREEN.r;
            startMarker.color.g = GREEN.g;
            startMarker.color.b = GREEN.b;
            startMarker.color.a = GREEN.a;
            startMarker.lifetime = ros::Duration(0);
            marker_pub.publish(startMarker);
        }

        if (!goalPoints.empty()) {
            visualization_msgs::Marker goalMarker;
            goalMarker.header.frame_id = "map";
            goalMarker.header.stamp = ros::Time::now();
            goalMarker.ns = "End_same_node";
            goalMarker.id = 0;
            goalMarker.type = visualization_msgs::Marker::POINTS;
            goalMarker.action = visualization_msgs::Marker::ADD;
            goalMarker.points = goalPoints;
            goalMarker.scale.x = 0.3;
            goalMarker.scale.y = 0.3;
            goalMarker.scale.z = 0.0;
            goalMarker.color.r = RED.r;
            goalMarker.color.g = RED.g;
            goalMarker.color.b = RED.b;
            goalMarker.color.a = RED.a;
            goalMarker.lifetime = ros::Duration(0);
            marker_pub.publish(goalMarker);
        }

        printf("可视化了 %zu 个起点连通点, %zu 个终点连通点\n", startPoints.size(), goalPoints.size());
    }

    if (parent[startIdx] == -1 || parent[goalIdx] == -1) {
        printf("并查集判断：起点或终点不可通行\n");
        return false;
    }

    bool isConnected = (find(startIdx) == find(goalIdx));
    if (isConnected) {
        printf("并查集判断：可通行！\n");
    } else {
        printf("并查集判断：不可通行！\n");
    }

    return isConnected;
}

 Node3D* aStar3D_H(Node3D& start, const Node3D& goal, Node3D* nodes3D, int width, int height,bool showflag,

  CollisionDetection& configurationSpace,
bool *way_finded,
int *planning_iterations_output,
double *path_cost_output) {

    int id = 0;
    if(!caculate_time){
      clearMarkers("");
      clearMarkers_openset("");

        publishMarker(start,GREEN,id,"path_points",0.5);
        id++;
        publishMarker(goal,PURPLE,id,"path_points",0.5);
        id++;
      }

    std::cout << "Starting 3D A* search..." << std::endl;
    std::cout << "Start: (" << start.getX() << ", " << start.getY()
              << ", " << start.getT() << ")" << std::endl;
    std::cout << "Goal:  (" << goal.getX() << ", " << goal.getY()
              << ", " << goal.getT() << ")" << std::endl;

    SamplingResult sample_table = SamplingResult::createWithDefaults();
    const auto hexa_sampling = sample_table.getHexahedronEndPoints();
    const auto time_sampling = sample_table.getTimeIntervalEndPoints();
    omni_w_max=sample_table.getomni_w_max();
    side_v=HybridAStar::Constants::V_max_vision;

    int totalNodes = width * height * Constants::positions * Constants::headings;

    for (int i = 0; i < totalNodes; i++) {
        nodes3D[i].reset();
        nodes3D[i].setG(0.0f);
        nodes3D[i].setH(0.0f);
        nodes3D[i].setPred(nullptr);

    }

    typedef boost::heap::binomial_heap<Node3D*,
          boost::heap::compare<CompareNodes>
          > priorityQueue;
    priorityQueue openSet;
    std::unordered_set<int> openSetIndices;

    int startIdx = start.setIdxM(start.getX(),start.getY(),start.getT(),width,height);
    if (startIdx < 0 || startIdx >= totalNodes) {
        std::cout << "Invalid start index: " << startIdx << std::endl;
        return nullptr;
    }
    start.set_index_neighbor(-1);

    nodes3D[startIdx] = start;
    nodes3D[startIdx].setG(0.0f);

    nodes3D[startIdx].setH(calculateHeuristic_M(start, goal,width,height));
    nodes3D[startIdx].open();

    openSet.push(&nodes3D[startIdx]);
    openSetIndices.insert(startIdx);

    int iterations = 0;
    int nodesExpanded = 0;

    Node3D* last_node ;

    while (!openSet.empty()

  ) {
        iterations++;

        Node3D* current = openSet.top();
        openSet.pop();

        last_node = current;

        int currentIdx = current->setIdxM(current->getX(),current->getY(),current->getT(),width,height);
        openSetIndices.erase(currentIdx);

        if (current->isClosed()) {
            continue;
        }

        current->close();
        nodesExpanded++;

        if (showflag && current != nullptr) {
            publishMarker_openset(*current,GREEN,id,"local_good_node",0.3);

            id++;

            printf("distance : (%.4f , %.4f, %.4f)\r\n\r\n",
              std::abs(current->getX()-goal.getX()),
              std::abs(current->getY()-goal.getY()),
              std::abs(current->getT()-goal.getT()));

        }

        if (isGoalReached(*current, goal)) {
            std::cout << "Goal reached!" << std::endl;
            std::cout << "Iterations: " << iterations << std::endl;
            std::cout << "Nodes expanded: " << nodesExpanded << std::endl;
            std::cout << "Path cost: " << current->getG() << std::endl;

            if(!caculate_time){
              plt_M.publishPath(current);
            }

            if (way_finded != nullptr) {
              *way_finded = true;
            }
            if (planning_iterations_output != nullptr) {
              *planning_iterations_output = iterations;
            }
            if (path_cost_output != nullptr) {
              *path_cost_output = current->getG();
            }

            return current;
        }

        for (int i=0;i<sample_table.getnumpoints();i++) {

            Eigen::Vector3d sample_P_local;
            if(Constants::fix_axil){
              sample_P_local=sample_table.get_ordi_EndPoints()[i];
            }
            else sample_P_local=sample_table.getTimeIntervalEndPoints()[i];

            Eigen::Vector2d sample_p_local=Eigen::Vector2d(sample_P_local[0],sample_P_local[1]);
            Eigen::Rotation2Dd rotation(current->getT());
            Eigen::Vector2d translation(current->getX(), current->getY());
            Eigen::Vector2d neighbor_i=rotation * sample_p_local + translation;
            double new_T = normalizeAngle(current->getT()+sample_P_local[2]);
            Eigen::Vector3d neighbor_i_={neighbor_i[0],neighbor_i[1],new_T};
            if(i==13 && iterations==3){

            }

            Node3D tempNeighbor(neighbor_i_[0], neighbor_i_[1], neighbor_i_[2], 0, 0, nullptr);

            int neighborIdx = tempNeighbor.setIdxM(tempNeighbor.getX(),tempNeighbor.getY(),tempNeighbor.getT(),width,height);

            if (neighborIdx < 0 || neighborIdx >= totalNodes) {
                continue;
            }

            if (!configurationSpace.isTraversable(&tempNeighbor)) {

                if(showflag&&(!caculate_time)){
                    publishMarker_openset(tempNeighbor,RED,id,"obs_points",0.5);
                    id++;
                }
                continue;
            }

            if (nodes3D[neighborIdx].isClosed()) {
                continue;
            }

            if(showflag && nodesExpanded==1&&(!caculate_time)){
                publishMarker_openset(tempNeighbor,BLUE,id,"neighbor_points",0.2);
                id++;
              }

            double d_g=sample_table.get_Tmax();
            int last_tip=current->get_index_neighbor();
            if(last_tip>=0 && last_tip<sample_table.getnumpoints()){
              if(Constants::fix_axil){
                d_g = sample_table.getneighbor_ordi_g_()[last_tip][i];
              }else{
                d_g = sample_table.getneighbor_g_()[last_tip][i];
              }
            }
            float newG = current->getG() + d_g;

            bool inOpenSet = openSetIndices.count(neighborIdx) > 0;

            if (!inOpenSet || newG < nodes3D[neighborIdx].getG()) {
                nodes3D[neighborIdx].setX(neighbor_i_[0]);
                nodes3D[neighborIdx].setY(neighbor_i_[1]);
                nodes3D[neighborIdx].setT(neighbor_i_[2]);
                nodes3D[neighborIdx].setG(newG);

                nodes3D[neighborIdx].setH(calculateHeuristic_M(nodes3D[neighborIdx],
                  goal,width,height));

            if(showflag){

                printf("neighbor [%d] : (%.4f , %.4f, %.4f)  g,h =(%.4f , %.4f)  f= %.4f \r\n",i,nodes3D[neighborIdx].getX(),nodes3D[neighborIdx].getY(),nodes3D[neighborIdx].getT()
                ,nodes3D[neighborIdx].getG(),nodes3D[neighborIdx].getH(),nodes3D[neighborIdx].getG()+nodes3D[neighborIdx].getH());
            }
              nodes3D[neighborIdx].setPred(current);
              nodes3D[neighborIdx].set_index_neighbor(i);
              if (!inOpenSet) {
                  nodes3D[neighborIdx].open();
                  openSet.push(&nodes3D[neighborIdx]);
                  openSetIndices.insert(neighborIdx);
              }
            }
        }

        if (iterations % 5000 == 0) {
            std::cout << "Progress - Iterations: " << iterations
                      << ", Expanded: " << nodesExpanded
                      << ", Open set size: " << openSet.size() << std::endl;
        }

    }

    std::cout << "No path found!" << std::endl;
    std::cout << "Total iterations: " << iterations << std::endl;
    std::cout << "Nodes expanded: " << nodesExpanded << std::endl;

    if (way_finded != nullptr) {
      *way_finded = false;
    }
    if (planning_iterations_output != nullptr) {
      *planning_iterations_output = iterations;
    }
    if (path_cost_output != nullptr) {
      *path_cost_output = last_node != nullptr ? last_node->getG() : 0.0;
    }
    return last_node;
}

std::vector<Node3D> reconstructPath(const Node3D& goalNode) {
    std::vector<Node3D> path;
    const Node3D* current = &goalNode;

    while (current != nullptr) {
        path.push_back(*current);
        current = current->getPred();
    }

    std::reverse(path.begin(), path.end());

    std::cout << "Path reconstructed with " << path.size() << " waypoints." << std::endl;

    if (path.size() > 1) {
        float totalDistance = 0.0f;
        float totalRotation = 0.0f;

        for (size_t i = 1; i < path.size(); i++) {
            float dx = path[i].getX() - path[i-1].getX();
            float dy = path[i].getY() - path[i-1].getY();
            totalDistance += std::sqrt(dx*dx + dy*dy);

            float dt = std::abs(normalizeAngle(path[i].getT() - path[i-1].getT()));
            totalRotation += std::min(dt, (float)(2*M_PI - dt));
        }

        std::cout << "Path length: " << totalDistance << " units" << std::endl;
        std::cout << "Total rotation: " << totalRotation << " radians" << std::endl;
    }

    return path;
}

void show_const_sampling(const std::vector<Eigen::Vector3d>& sampling_points,const Color color,const std::string& ns) {

    int id = 1000;

    for (const auto& point : sampling_points) {
        Node3D node_to_display(
            point.x(),
            point.y(),
            point.z(),
            0.0,
            0.0,
            nullptr,
            0
        );

        publishMarker(node_to_display, color, id++, ns, 0.1);
    }
    std::cout << "采样点显示完毕。" << std::endl;
}

struct CompareNodesH {

  bool operator()(const Node3D* lhs, const Node3D* rhs) const {
    return lhs->getH() > rhs->getH();
  }

  bool operator()(const Node2D* lhs, const Node2D* rhs) const {
    return lhs->getH() > rhs->getH();
  }
};

NodeAWS* Algorithm::hybridAStar_aws(NodeAWS& start,
                               const NodeAWS& goal,
                               NodeAWS* nodes3D,
                               Node2D* nodes2D,
                               int width,
                               int height,
                               CollisionDetection& configurationSpace,
                               float* dubinsLookup,
                               Visualize& visualization,
                              bool showflag) {
  releaseAwsSearchMemory();

  printf("使用的算法：CAWS（别人的算法）\r\n");

  int iPred, iSucc;
  float newG;

  int iterations = 0;

  ros::Duration d(0.003);

  typedef boost::heap::binomial_heap<NodeAWS*,
          boost::heap::compare<CompareNodes>
          > priorityQueue;
  priorityQueue O;
  typedef boost::heap::binomial_heap<NodeAWS*,
          boost::heap::compare<CompareNodesH>
          > priorityQueueH;
  priorityQueueH O_H;

    ros::NodeHandle nh;
    initPathPublisher(nh);

  const bool start_traversable_aws = configurationSpace.configurationTest_polygon(start.getX(), start.getY(), start.getT());
  const bool goal_traversable_aws = configurationSpace.configurationTest_polygon(goal.getX(), goal.getY(), goal.getT());
  std::cout << "CAWS traversability check: start=" << start_traversable_aws
            << " (" << start.getX() << ", " << start.getY() << ", " << start.getT() << ")"
            << ", goal=" << goal_traversable_aws
            << " (" << goal.getX() << ", " << goal.getY() << ", " << goal.getT() << ")"
            << std::endl;
  auto print_collision_reason = [&](const char* label, const NodeAWS& node) {
    auto occupied = configurationSpace.returnpolgen(node.getX(), node.getY(), node.getT(), false);
    auto grid = configurationSpace.get_grid();
    if (!grid) {
      std::cout << "CAWS " << label << " collision reason: grid is null" << std::endl;
      return;
    }
    for (const auto& point : occupied) {
      const int cX = static_cast<int>(point.x());
      const int cY = static_cast<int>(point.y());
      if (cX < 0 || static_cast<unsigned int>(cX) >= grid->info.width ||
          cY < 0 || static_cast<unsigned int>(cY) >= grid->info.height) {
        std::cout << "CAWS " << label << " collision reason: out_of_grid cell=("
                  << cX << ", " << cY << ") map_size=("
                  << grid->info.width << ", " << grid->info.height << ")" << std::endl;
        return;
      }
      const int idx = cY * grid->info.width + cX;
      if (grid->data[idx]) {
        std::cout << "CAWS " << label << " collision reason: occupied cell=("
                  << cX << ", " << cY << ") value=" << static_cast<int>(grid->data[idx])
                  << std::endl;
        return;
      }
    }
    std::cout << "CAWS " << label << " collision reason: no occupied cell found in returnpolgen" << std::endl;
  };

  if((!start_traversable_aws) || (!goal_traversable_aws) ){

    if (!start_traversable_aws) { print_collision_reason("start", start); }
    if (!goal_traversable_aws) { print_collision_reason("goal", goal); }
    printf("起点或终点不可通行!  \r\n");
    return nullptr;
  }else{
    printf("起点和终点可通行!  \r\n");

  }

    int id = 0;
    if(!caculate_time){
      clearMarkers("");
      clearMarkers_openset("");

      }

  const auto sampling_table = AWSConstructSamplingTable(
                        Constants_aws::r_sample_num,
                        Constants_aws::psi_sample_num,

                        linspace(-M_PI_4, M_PI_4,
                                  Constants_aws::omega_sample_num),
                        sqrt(2*Constants_aws::cellSize*Constants_aws::cellSize)/2
                        );

  int dir = sampling_table.size();

  updateH(start, goal, nodes2D, dubinsLookup, width, height,
          configurationSpace, visualization);

  start.open();
  start.sampling_table = &sampling_table;

  O.push(&start);
  iPred = start.setIdxM(start.getX(),start.getY(),start.getT(),width, height);

  nodes3D[iPred] = start;

  NodeAWS* nPred;
  NodeAWS* nSucc;

  int nodesExpanded = 0;

  while (!O.empty()) {

    nPred = O.top();
    O_H.push(nPred);

    O.pop();

    iPred = nPred->setIdxM(nPred->getX(),nPred->getY(),nPred->getT(),width, height);
    iterations++;

    if (Constants_aws::visualization) {
      visualization.publishNode3DPoses(*nPred);
      visualization.publishNode3DPose(*nPred);

    }

    if (nodes3D[iPred].isClosed()) {
      continue;
    }

    nodesExpanded++;

    if (showflag ) {
        Node3D tempNeighbor1(nodes3D[iPred].getX(), nodes3D[iPred].getY(), nodes3D[iPred].getT(), 0, 0, nullptr);
        publishMarker_openset(tempNeighbor1,GREEN,id,"local_good_node",0.3);

        id++;
    }

    if (nodes3D[iPred].isOpen()) {

      nodes3D[iPred].close();

      if (*nPred == goal

      ) {

        std::cout<<"goal found" <<std::endl;
        std::cout<<nPred->getPred()<< " , "<< nPred <<std::endl;

        return O_H.top();
      }

      for (int i = 0; i < dir; ++i) {

        nPred->sampling_table = &sampling_table;
        nSucc = nPred->createSuccessor(i);

        if(showflag && nodesExpanded==1){
            Node3D tempNeighbor2(nSucc->getX(), nSucc->getY(), nSucc->getT(), 0, 0, nullptr);
            publishMarker_openset(tempNeighbor2,BLUE,id,"local_good_node",0.2);
            id++;
          }

        if(nSucc==nPred){
          std::cout << "looping at start"<<std::endl;
          return nSucc;
        }

        iSucc = nSucc->setIdxM(nSucc->getX(),nSucc->getY(),nSucc->getT(),width, height);

        if (!nSucc->isOnGrid(width, height) ||
            !configurationSpace.isTraversable(nSucc))
        {

          Node3D tempNeighbor(nSucc->getX(), nSucc->getY(), nSucc->getT(), 0, 0, nullptr);

          delete nSucc;
          continue;
        }

        nSucc->updateG();
        newG = nSucc->getG();

        if ((nodes3D[iSucc].isClosed() || nodes3D[iSucc].isOpen())
            && iPred != iSucc
            && newG >= nodes3D[iSucc].getG())
        {

          delete nSucc;
          continue;
        }
        updateH(*nSucc, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization);

        if(nSucc==nSucc->getPred()){
          std::cout << "looping after H update"<<std::endl;
          return nSucc;
        }
        if (iSucc == iPred &&
            nSucc->getC() >
            nPred->getC() - Constants_aws::tieBreaker)
        {

          delete nSucc;
          continue;
        }
        if (nSucc->getPred() == nSucc) {
          std::cout << "looping in exploration"<<std::endl;
          return nullptr;
        }

        nSucc->open();
        nodes3D[iSucc] = *nSucc;
        O.push(nSucc);
        awsSuccessorPool.emplace_back(nSucc);
      }
    }
    if (iterations % 5000 == 0) {
        std::cout << "Progress - Iterations: " << iterations
                  << ", Expanded: " << nodesExpanded
                  << ", Open set size: " << O.size() << std::endl;
    }

  }

    std::cout << "No path found!" << std::endl;
    std::cout << "Total iterations: " << iterations << std::endl;
    std::cout << "Nodes expanded: " << nodesExpanded << std::endl;
    return nullptr;
}

NodeAWS* dubinsShot(NodeAWS& start, const NodeAWS& goal, CollisionDetection& configurationSpace) {

  double q0[] = { start.getX(), start.getY(), start.getT() };

  double q1[] = { goal.getX(), goal.getY(), goal.getT() };

  DubinsPath path;

  dubins_init(q0, q1, Constants_aws::r, &path);

  int i = 0;
  float x = 0.f;
  float length = dubins_path_length(&path);

  NodeAWS* dubinsNodes = new NodeAWS [(int)(length / Constants_aws::dubinsStepSize) + 1];

  x += Constants_aws::dubinsStepSize;
  while (x <  length) {
    double q[3];
    dubins_path_sample(&path, x, q);
    dubinsNodes[i].setX(q[0]);
    dubinsNodes[i].setY(q[1]);
    dubinsNodes[i].setT(Helper::normalizeHeadingRad(q[2]));

    if (configurationSpace.isTraversable(&dubinsNodes[i])) {

      if (i > 0) {
        dubinsNodes[i].setPred(&dubinsNodes[i - 1]);
      } else {
        dubinsNodes[i].setPred(&start);
      }

      if (&dubinsNodes[i] == dubinsNodes[i].getPred()) {
        std::cout << "looping shot";
      }

      x += Constants_aws::dubinsStepSize;
      i++;
    } else {

      delete [] dubinsNodes;
      return nullptr;
    }
  }

  return &dubinsNodes[i - 1];
}

template<typename T>
std::vector<float> linspace(T start_in, T end_in, int num_in)
{

  std::vector<float> linspaced;

  float start = static_cast<float>(start_in);
  float end = static_cast<float>(end_in);
  float num = static_cast<float>(num_in);

  if (num == 0) { return linspaced; }
  if (num == 1)
    {
      linspaced.push_back(start);
      return linspaced;
    }

  float delta = (end - start) / (num - 1);

  for(int i=0; i < num-1; ++i)
    {
      linspaced.push_back(start + delta * i);
    }
  linspaced.push_back(end);

  return linspaced;
}

Eigen::Matrix<float,2,2> R(float theta){
  Eigen::Matrix<float,2,2> R;
  R << cos(theta), -sin(theta),
       sin(theta), cos(theta);
  return R;
}

int sign(double val) {
    if (val > 0) {
        return 1;
    } else if (val < 0) {
        return -1;
    } else {
        return 0;
    }
}

std::vector<std::vector<float>> AWSConstructSamplingTable(int r_samples, int psi_samples, std::vector<float> omega_sample_list, float delta_cur){
  float epsilon = 1e-5;

  float delta_sample_r = M_PI_2/r_samples - epsilon;
  float delta_psi = 2*M_PI/psi_samples;

  std::vector<float> wheel_positions_x=Constants_aws::wheel_positions_x;
  std::vector<float> wheel_positions_y=Constants_aws::wheel_positions_y;

  std::vector<std::vector<float>> samplingTable;
  for (int i = 0; i <= r_samples; ++i){
    for (int j = 0; j <= psi_samples; ++j){
      float r_scale = tan(delta_sample_r*i + epsilon);
      float psi = -M_PI+delta_psi*j;
      float r_0 = -r_scale*cos(psi);
      float r_1 = -r_scale*sin(psi);
      bool infeasible = false;

      for (int k = 0; k < Constants_aws::wheel_num; ++k){
        float steer_ulim_p_pi_d_2 =
          Constants_aws::wheel_steer_limits_up[k] + M_PI_2;
        float steer_llim_p_pi_d_2 =
          Constants_aws::wheel_steer_limits_low[k] + M_PI_2;
        float x = wheel_positions_x[k];
        float y = wheel_positions_y[k];
        float r_w_0 = r_0 + x;
        float r_w_1 = r_1 + y;

        if (Constants_aws::has_steer_limit &&
            (r_w_0*sin(steer_ulim_p_pi_d_2) - r_w_1*cos(steer_ulim_p_pi_d_2))*
            (r_w_0*sin(steer_llim_p_pi_d_2) - r_w_1*cos(steer_llim_p_pi_d_2))
            > 0

            ){
          std::cout << "steer limit violated: "<<
            atan2(r_w_1, r_w_0)/M_PI*180-90 <<r_scale<< std::endl;
          infeasible = true;
          break;
        }
      }
      if (infeasible)
      {
        continue;
      }

      for (float omega: omega_sample_list){
        float max_omega = delta_cur/r_scale;
        float prac_omega = sign(omega)*
                std::min(abs(omega), abs(max_omega));
        float v_x = -prac_omega*r_1;
        float v_y = prac_omega*r_0;
        Eigen::Vector2f r(r_0, r_1);
        Eigen::Vector2f d_xy = R(prac_omega)*r - r;
        float d_x = d_xy(0);
        float d_y = d_xy(1);
        std::vector<float> combo_pos = {d_x, d_y, prac_omega,
                                        v_x, v_y, prac_omega};

        infeasible = false;
        for (int m = 0; m < samplingTable.size(); ++m){
          if (abs(combo_pos[0]-samplingTable[m][0])<1e-1 &&
              abs(combo_pos[1]-samplingTable[m][1])<1e-1 &&
              abs(combo_pos[2]-samplingTable[m][2])<1e-1){
            infeasible = true;
            break;
          }
        }
        if (infeasible) continue;

        samplingTable.push_back(combo_pos);

        if (omega == max_omega) break;
      }
    }
  }

  std::vector<std::vector<float>> uniqueTable;
  std::unique_copy(samplingTable.begin(),
                    samplingTable.end(),
                    std::back_inserter(uniqueTable));
  samplingTable.assign(uniqueTable.begin(), uniqueTable.end());
  return samplingTable;
}
