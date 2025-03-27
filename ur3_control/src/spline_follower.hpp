#ifndef SPLINE_FOLLOWER_HPP
#define SPLINE_FOLLOWER_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <iostream>
#include <rclcpp/parameter_client.hpp>

using json = nlohmann::json;

class SplineFollower : public rclcpp::Node {
public:
    SplineFollower();

    // Declare public member functions for use in the main
    void setSafeStartPose();
    void addGroundPlane();
    bool loadSplines();
    double calculateAverageCanvasHeight();
    const std::vector<geometry_msgs::msg::Pose> computeLinearInterpolationPath(
            const geometry_msgs::msg::Pose& start_pose, 
            const geometry_msgs::msg::Pose& end_pose, 
            int num_waypoints);

    // Declare and define state enum for state machine
    enum class State {
        INIT,
        MOVE_TO_INTERMEDIATE_POS,
        MOVE_TO_CANVAS,
        MOVE_THROUGH_DRAWING_TRAJECTORY,
        MOVE_OFF_CANVAS,
        STOP
    };

    // Declare public member variables for use in main
    geometry_msgs::msg::Pose safe_start_pose_;
    State state_;
    nlohmann::json spline_data_;
    // geometry_msgs::msg::Pose safe_end_pose_;
    // std::vector<geometry_msgs::msg::Pose> current_trajectory_;
    // const double LIFT_HEIGHT = 0.05;  // 50mm lift height
    size_t current_spline_index_;
    // std::vector<json> remaining_splines_;
    geometry_msgs::msg::Pose intermediate_pose_;
    geometry_msgs::msg::Pose canvas_pose_;

};

#endif  // SPLINE_FOLLOWER_HPP
