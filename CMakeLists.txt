cmake_minimum_required(VERSION 3.0.2)
project(dual_chaser)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Debug)
find_package(Eigen3 REQUIRED NO_MODULE)


find_package(catkin REQUIRED roscpp octomap_ros octomap_server pcl_ros tf chasing_utils)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dual_chaser
#  CATKIN_DEPENDS chasing_utils
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
        ${catkin_INCLUDE_DIRS}
)

add_executable(preplanner_node exe/PreplannerNode.cpp src/Preplanner.cpp src/Wrapper.cpp src/ScoreField.cpp)
target_link_libraries(preplanner_node  ${catkin_LIBRARIES})




