#include <memory>
#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

// Function to get pose based on command line argument
geometry_msgs::msg::Pose get_pose(const std::string& pose_name) {
    geometry_msgs::msg::Pose pose;
    if (pose_name == "pick") {
        pose.position.x = 0.5; // Placeholder values
        pose.position.y = 0.0;
        pose.position.z = -0.13;
        pose.orientation.w = 0.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 1.0;
        pose.orientation.z = 0.0;
        
    } else if (pose_name == "home") {
        pose.position.x = 0.25;
        pose.position.y = 0.0;
        pose.position.z = 0.6;
        pose.orientation.w = 0.0;
        pose.orientation.x = 0.7071;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.7071;
        
    } else if (pose_name == "place") {
        pose.position.x = 0.5; // Placeholder values
        pose.position.y = 0.0;
        pose.position.z = -0.13;
        pose.orientation.w = 0.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 1.0;
        pose.orientation.z = 0.0;
    } else {
        std::cerr << "Invalid pose name: " << pose_name << std::endl;
        std::exit(1);
    }
    return pose;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <pose_name>" << std::endl;
        return 1;
    }
    std::string pose_name = argv[1];

    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_moveit");

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "panda_arm");

    // Set a target Pose based on command line argument
    auto const target_pose = get_pose(pose_name);
    move_group_interface.setPoseTarget(target_pose);

    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success) {
        move_group_interface.execute(plan);
    } else {
        RCLCPP_ERROR(logger, "Planning failed!");
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}

