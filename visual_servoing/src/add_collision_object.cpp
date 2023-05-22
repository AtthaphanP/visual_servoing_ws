#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/ColorRGBA.h>

using namespace moveit::planning_interface;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_robot");
    ros::NodeHandle nh;

    // Create a MoveGroupInterface instance
    MoveGroupInterface move_group("manipulator");

    // Set the planning time and number of planning attempts
    move_group.setPlanningTime(5.0);
    move_group.setNumPlanningAttempts(10);

    // Create a PlanningSceneInterface instance
    PlanningSceneInterface planning_scene_interface;

    // Create and add the first collision object
    moveit_msgs::CollisionObject collision_object1;
    collision_object1.header.frame_id = move_group.getPlanningFrame();
    collision_object1.id = "Table";
    shape_msgs::SolidPrimitive Table;
    Table.type = shape_msgs::SolidPrimitive::BOX;
    Table.dimensions = {1.01, 0.85, 0.79};
    geometry_msgs::Pose pose1;
    pose1.position.x = 0.0;
    pose1.position.y = 0.0;
    pose1.position.z = -0.4;
    pose1.orientation.w = 1.0;
    collision_object1.primitives.push_back(Table);
    collision_object1.primitive_poses.push_back(pose1);

    planning_scene_interface.applyCollisionObject(collision_object1);
    ROS_INFO("Added Table into the world");

    // Create and add the second collision object
    moveit_msgs::CollisionObject collision_object2;
    collision_object2.header.frame_id = move_group.getPlanningFrame();
    collision_object2.id = "Controller";
    shape_msgs::SolidPrimitive Controller;
    Controller.type = shape_msgs::SolidPrimitive::BOX;
    Controller.dimensions = {0.7, 0.7, 0.5};
    geometry_msgs::Pose pose2;
    pose2.position.x = 0.0;
    pose2.position.y = -0.85;
    pose2.position.z = 0.25;
    pose2.orientation.w = 1.0;
    collision_object2.primitives.push_back(Controller);
    collision_object2.primitive_poses.push_back(pose2);
    planning_scene_interface.applyCollisionObject(collision_object2);
    ROS_INFO("Added Controller into the world");

    // Create and add the third collision object
    moveit_msgs::CollisionObject collision_object3;
    collision_object3.header.frame_id = move_group.getPlanningFrame();
    collision_object3.id = "wall";
    shape_msgs::SolidPrimitive wall;
    wall.type = shape_msgs::SolidPrimitive::BOX;
    wall.dimensions = {0.2, 2.0, 2.0};
    geometry_msgs::Pose pose3;
    pose3.position.x = -0.4;
    pose3.position.y = 0.0;
    pose3.position.z = 1.0;
    pose3.orientation.w = 1.0;
    collision_object3.primitives.push_back(wall);
    collision_object3.primitive_poses.push_back(pose3);
    planning_scene_interface.applyCollisionObject(collision_object3);
    ROS_INFO("Added wall into the world");

    ros::spin();
    return 0;
}
