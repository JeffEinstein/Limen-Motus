cmake_minimum_required(VERSION 2.8.3)
project(limen_motus)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs tf message_generation sensor_msgs cv_bridge nav_msgs geometry_msgs visualization_msgs tf2 tf2_ros)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)
find_package(ompl REQUIRED)

add_message_files(FILES car_control.msg)
generate_messages(DEPENDENCIES std_msgs)

set(PLANNER_SOURCES
  src/algorithm.cpp
  src/bucketedqueue.cpp
  src/collisiondetection.cpp
  src/dubins.cpp
  src/dynamicvoronoi.cpp
  src/node2d.cpp
  src/node3d.cpp
  src/node_aws.cpp
  src/path.cpp
  src/planner.cpp
  src/show.cpp
  src/SamplingM.cpp
  src/visualize.cpp
  src/vplanner.cpp
)

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs message_runtime)

include_directories(include /usr/include/python3.8 /usr/include/eigen3 /usr/lib/python3/dist-packages/numpy/core/include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})

add_executable(limen_motus src/main.cpp ${PLANNER_SOURCES})
add_dependencies(limen_motus ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(limen_motus ${catkin_LIBRARIES} ${OMPL_LIBRARIES} python3.8)

install(TARGETS limen_motus RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch maps DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
