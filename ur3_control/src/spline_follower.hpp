#ifndef SPLINE_FOLLOWER_HPP
#define SPLINE_FOLLOWER_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include <memory>

using json = nlohmann::json;

class SplineFollower : public rclcpp::Node {
public:
    SplineFollower();
    void on_activate();

private:
    enum class State {
        INIT,
        GEN_INTERMEDIATE_TRAJ,
        MOVE_TO_INTERMEDIATE_POS,
        GEN_DRAWING_TRAJECTORY,
        MOVE_THROUGH_DRAWING_TRAJECTORY,
        STOP
    };

    State state_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    nlohmann::json spline_data_;
    geometry_msgs::msg::Pose safe_start_pose_;
    geometry_msgs::msg::Pose safe_end_pose_;
    std::vector<geometry_msgs::msg::Pose> current_trajectory_;
    const double LIFT_HEIGHT = 0.05;  // 50mm lift height
    size_t current_spline_index_;

    void runStateMachine();
    void addGroundPlane();
    bool loadSplines();
    bool initializeSafePoses();
    geometry_msgs::msg::Pose parsePose(const YAML::Node& node);
    void setSafeStartPose();
    void generateIntermediateTrajectory();
    void generateDrawingTrajectory();
    bool executeTrajectory(const std::vector<geometry_msgs::msg::Pose>& waypoints);
    void executePoseTarget(const geometry_msgs::msg::Pose& target_pose, const std::string& description);
    void moveToSafeEndPose();
    void stopExecution();

    // Stores the remaining splines to process
    std::vector<json> remaining_splines_;

    // Stores the intermediate and drawing trajectories
    moveit_msgs::msg::RobotTrajectory intermediate_trajectory_;
    moveit_msgs::msg::RobotTrajectory drawing_trajectory_;

    // Helper function for getting waypoint pose
    geometry_msgs::msg::Pose getWaypointPose(const json& waypoint, double lift);
};

#endif  // SPLINE_FOLLOWER_HPP
