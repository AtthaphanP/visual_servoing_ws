// #include <ros/ros.h>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <geometry_msgs/PoseStamped.h>

// static const std::string PLANNING_GROUP = "manipulator";
// double x, y, z, qx, qy, qz, qw;


// void poseCallback()
// {   
//         // Check if all variables are 0.0 and return if true
//     if (x == 0.0 && y == 0.0 && z == 0.0 && qx == 0.0 && qy == 0.0 && qz == 0.0) 
//     {
//         return;
//     }
//     ROS_INFO("fUNCTION posecallback");
//     ROS_INFO("Received Aruco_ros pose: x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f", x, y, z, qx, qy, qz, qw);

//     moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
//     move_group.setPlanningTime(3.0);
//     move_group.setNumPlanningAttempts(5);
//     move_group.setMaxVelocityScalingFactor(0.8);        // Speed 80%
//     move_group.setMaxAccelerationScalingFactor(0.8);    // Speed 80%
//     move_group.setGoalTolerance(0.001);

//     geometry_msgs::Pose target_pose;
//     target_pose.position.x = x;
//     target_pose.position.y = y;
//     target_pose.position.z = z;
//     target_pose.orientation.x = qx;
//     target_pose.orientation.y = qy;
//     target_pose.orientation.z = qz;
//     target_pose.orientation.w = qw;

//     move_group.setPoseTarget(target_pose);

//     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//     bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if (success) {
//         move_group.execute(my_plan);
//         ROS_INFO("Move to target pose success!");
//     } else {
//         ROS_WARN("Move to target pose failed!");
//     }
//     x = 0.0;
//     y = 0.0;    
//     z = 0.0;    
//     qx = 0.0;   
//     qy = 0.0;   
//     qz = 0.0;   
//     qw = 0.0;   
//     ROS_INFO("end loop");
// }

// void callmsgs(const geometry_msgs::PoseStamped::ConstPtr& msg)
// {   
//     // Extract position information from the PoseStamped message
//     x = std::floor((msg->pose.position.x - 0.4) * 1000) / 1000;
//     y = std::floor(msg->pose.position.y * 1000) / 1000;
//     z = std::floor(msg->pose.position.z * 1000) / 1000;

//     // Extract orientation information from the PoseStamped message
//     qx = std::floor(msg->pose.orientation.x * 1000) / 1000;
//     qy = std::floor(msg->pose.orientation.y * 1000) / 1000;
//     qz = std::floor(msg->pose.orientation.z * 1000) / 1000;
//     qw = std::floor((msg->pose.orientation.w + 0.432050) * 1000) / 1000;
// }

// int main(int argc, char** argv)
// {   
//     ROS_INFO("START");
//     ros::init(argc, argv, "move_to_aruco_pose");
//     ros::NodeHandle nh;

//     ros::AsyncSpinner spinner(2); // Use async spinner instead of multi-threaded
//     spinner.start();

//     ros::Subscriber sub = nh.subscribe("aruco_single/pose", 1000, callmsgs);
    
//     while (ros::ok()) 
//     {   
//         poseCallback();
//     }

//     ROS_INFO("END");
//     ros::waitForShutdown();
//     return 0;
// }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>

void callmsgs(const geometry_msgs::PoseStamped::ConstPtr& msg)
{   
    // Extract position information from the PoseStamped message
    double x = std::floor((msg->pose.position.x - 0.4) * 1000) / 1000;
    double y = std::floor(msg->pose.position.y * 1000) / 1000;
    double z = std::floor(msg->pose.position.z * 1000) / 1000;

    // Extract orientation information from the PoseStamped message
    double qx = std::floor(msg->pose.orientation.x * 1000) / 1000;
    double qy = std::floor(msg->pose.orientation.y * 1000) / 1000;
    double qz = std::floor(msg->pose.orientation.z * 1000) / 1000;
    double qw = std::floor((msg->pose.orientation.w + 0.432050) * 1000) / 1000;

    // Check if all variables are 0.0 and return if true
    if (x == 0.0 && y == 0.0 && z == 0.0 && qx == 0.0 && qy == 0.0 && qz == 0.0)
    {
        return;
    }

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setPlanningTime(3.0);
    move_group.setNumPlanningAttempts(5);
    move_group.setMaxVelocityScalingFactor(0.8);        // Speed 80%
    move_group.setMaxAccelerationScalingFactor(0.8);    // Speed 80%
    move_group.setGoalTolerance(0.001);

    geometry_msgs::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    target_pose.orientation.x = qx;
    target_pose.orientation.y = qy;
    target_pose.orientation.z = qz;
    target_pose.orientation.w = qw;

    move_group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        move_group.execute(my_plan);
        ROS_INFO("Move to target pose success!");
    } else {
        ROS_WARN("Move to target pose failed!");
    }
}

int main(int argc, char** argv)
{   
    ROS_INFO("START");
    ros::init(argc, argv, "move_to_aruco_pose");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2); // Use async spinner instead of multi-threaded
    spinner.start();

    ros::Subscriber sub = nh.subscribe("aruco_single/pose", 50, callmsgs);

    ROS_INFO("END");
    ros::waitForShutdown();
    return 0;
}