#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <vector>

using json = nlohmann::json;
using namespace std::chrono_literals;

class SplineFollower : public rclcpp::Node {
public:
    SplineFollower() : Node("spline_follower") {
        RCLCPP_INFO(this->get_logger(), "SplineFollower node created.");
    }

    void on_activate() {
        // Now we safely call shared_from_this() after the node is fully constructed
        auto node_shared_ptr = shared_from_this();
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_shared_ptr, "manipulator");

        RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized.");
        
        move_group_->setPlannerId("RRTConnectkConfigDefault");
        move_group_->setMaxVelocityScalingFactor(1.0);
        move_group_->setMaxAccelerationScalingFactor(1.0);

        if (!loadSplines()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load splines. Exiting...");
            return;
        }
        followSplines();
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    json spline_data_;

    bool loadSplines() {
        std::ifstream file("/home/jarred/git/DalESelfEBot/ur3_control/config/drawing_path.json");
        if (!file) {
            RCLCPP_ERROR(this->get_logger(), "Could not open spline JSON file.");
            return false;
        }

        file >> spline_data_;
        if (spline_data_["splines"].empty()) {
            RCLCPP_ERROR(this->get_logger(), "No splines found in JSON!");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %lu splines.", spline_data_["splines"].size());
        return true;
    }

    void followSplines() {
        for (const auto& spline : spline_data_["splines"]) {
            std::vector<geometry_msgs::msg::Pose> waypoints;

            for (const auto& waypoint : spline["waypoints"]) {
                if (waypoint.size() < 3) {
                    RCLCPP_WARN(this->get_logger(), "Skipping invalid waypoint.");
                    continue;
                }

                geometry_msgs::msg::Pose pose;
                pose.position.x = waypoint[0];
                pose.position.y = waypoint[1];
                pose.position.z = waypoint[2];

                // Set a fixed orientation (UR3e TCP pointing downwards)
                pose.orientation.x = 0.0;
                pose.orientation.y = 1.0;
                pose.orientation.z = 0.0;
                pose.orientation.w = 0.0;

                waypoints.push_back(pose);
            }

            executeTrajectory(waypoints);
        }
    }

    void executeTrajectory(const std::vector<geometry_msgs::msg::Pose>& waypoints) {
        if (waypoints.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No valid waypoints to execute.");
            return;
        }

        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

        if (fraction < 0.95) {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute full Cartesian path (%.2f%%).", fraction * 100);
            return;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        RCLCPP_INFO(this->get_logger(), "Executing spline trajectory...");
        move_group_->execute(plan);
        RCLCPP_INFO(this->get_logger(), "Trajectory execution completed.");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SplineFollower>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    RCLCPP_INFO(node->get_logger(), "Activating SplineFollower...");
    node->on_activate();  // Initialize MoveGroupInterface safely

    executor.spin();
    rclcpp::shutdown();
    return 0;
}