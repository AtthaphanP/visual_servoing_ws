cmake_minimum_required(VERSION 3.0.2)
project(visual_servoing)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
)

find_package(Boost REQUIRED COMPONENTS system thread)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp moveit_core moveit_ros_planning moveit_ros_planning_interface
  DEPENDS Boost
)

## Specify directories to include when compiling
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${MOVEIT_INCLUDE_DIRS}
)

## Build the robot_arm_controller node
add_executable(robot_arm_controller src/robot_arm_controller.cpp)
add_executable(add_collision_object src/add_collision_object.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(robot_arm_controller 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_link_libraries(add_collision_object 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)