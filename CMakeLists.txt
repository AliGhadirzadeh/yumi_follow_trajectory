cmake_minimum_required(VERSION 2.8.3)
project(yumi_follow_trajectory)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  kdl_wrapper
  roscpp
)
find_package(orocos_kdl REQUIRED)

catkin_package(
  DEPENDS orocos_kdl system_lib
  CATKIN_DEPENDS  kdl_parser kdl_wrapper roscpp
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${orocos_kdl_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})
find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})

add_executable(follow_joint_trajectory_node src/follow_joint_trajectory.cpp)
add_executable(yumi_pushing_node src/yumi_pushing.cpp)
target_link_libraries(follow_joint_trajectory_node  ${catkin_LIBRARIES} )
target_link_libraries(yumi_pushing_node  ${catkin_LIBRARIES} )
