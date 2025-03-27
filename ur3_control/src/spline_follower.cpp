#include "spline_follower.hpp"
#include <fstream>
#include <thread>

using namespace std::chrono_literals;
using json = nlohmann::json;

SplineFollower::SplineFollower() : Node("spline_follower"), state_(State::INIT), current_spline_index_(0) {
    RCLCPP_INFO(this->get_logger(), "SplineFollower node created.");
}

void SplineFollower::addGroundPlane() {
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::msg::CollisionObject ground;
    ground.header.frame_id = "world";
    ground.id = "ground_plane";

    shape_msgs::msg::SolidPrimitive ground_shape;
    ground_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
    ground_shape.dimensions = {10.0, 10.0, 0.01};

    geometry_msgs::msg::Pose ground_pose;
    ground_pose.position.z = -0.01;

    ground.primitives.push_back(ground_shape);
    ground.primitive_poses.push_back(ground_pose);
    ground.operation = ground.ADD;

    planning_scene_interface.applyCollisionObject(ground);
}

bool SplineFollower::loadSplines() {
    std::ifstream file("/home/jarred/git/DalESelfEBot/ur3_control/config/circle_waypoints.json");
    if (!file) {
        RCLCPP_ERROR(this->get_logger(), "Could not open spline JSON file.");
        return false;
    }

    file >> spline_data_;
    return !spline_data_["splines"].empty();
}

void SplineFollower::setSafeStartPose() {
    // Define the YAML file path
    std::string yaml_file = "/home/jarred/git/DalESelfEBot/ur3_localisation/config/params.yaml";

    // Open the YAML file
    std::ifstream file(yaml_file);
    if (!file) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open localization file: %s", yaml_file.c_str());
        return;
    }

    // Parse the YAML file
    YAML::Node config = YAML::Load(file);
    file.close();

    // Ensure that corner positions exist in the YAML file
    if (!config["corner_positions"] || config["corner_positions"].size() != 4) {
        RCLCPP_ERROR(this->get_logger(), "Invalid or missing corner_positions in params.yaml");
        return;
    }

    // Compute the center position of the rectangle
    double x_sum = 0.0, y_sum = 0.0;
    for (const auto& corner : config["corner_positions"]) {
        x_sum += corner["x"].as<double>();
        y_sum += corner["y"].as<double>();
    }

    safe_start_pose_.position.x = x_sum / 4.0;
    safe_start_pose_.position.y = y_sum / 4.0;
    safe_start_pose_.position.z = 0.1;  // 100mm above workspace

    // Orientation facing downward (Quaternion for downward orientation)
    safe_start_pose_.orientation.x = 0.0;
    safe_start_pose_.orientation.y = 1.0;  // Pointing downward
    safe_start_pose_.orientation.z = 0.0;
    safe_start_pose_.orientation.w = 0.0;

    RCLCPP_INFO(this->get_logger(), "Safe start pose set at (%.3f, %.3f, %.3f) with downward orientation.",
                safe_start_pose_.position.x, safe_start_pose_.position.y, safe_start_pose_.position.z);
}


const std::vector<geometry_msgs::msg::Pose> SplineFollower::computeLinearInterpolationPath(
    const geometry_msgs::msg::Pose& start_pose, 
    const geometry_msgs::msg::Pose& end_pose, 
    int num_waypoints) 
{
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(num_waypoints);

    for (int i = 0; i <= num_waypoints; i++) {
        double t = static_cast<double>(i) / static_cast<double>(num_waypoints);

        geometry_msgs::msg::Pose interpolated_pose;

        // Linear interpolation for position (X, Y, Z)
        interpolated_pose.position.x = start_pose.position.x + t * (end_pose.position.x - start_pose.position.x);
        interpolated_pose.position.y = start_pose.position.y + t * (end_pose.position.y - start_pose.position.y);
        interpolated_pose.position.z = start_pose.position.z + t * (end_pose.position.z - start_pose.position.z);

        // Spherical Linear Interpolation (SLERP) for orientation
        tf2::Quaternion start_q(
            start_pose.orientation.x, start_pose.orientation.y, 
            start_pose.orientation.z, start_pose.orientation.w);
        tf2::Quaternion end_q(
            end_pose.orientation.x, end_pose.orientation.y, 
            end_pose.orientation.z, end_pose.orientation.w);
        tf2::Quaternion interpolated_q = start_q.slerp(end_q, t);
        interpolated_pose.orientation.x = interpolated_q.x();
        interpolated_pose.orientation.y = interpolated_q.y();
        interpolated_pose.orientation.z = interpolated_q.z();
        interpolated_pose.orientation.w = interpolated_q.w();

        waypoints.push_back(interpolated_pose);
    }

    return waypoints;
}

double SplineFollower::calculateAverageCanvasHeight() {
    std::string yaml_path = "/home/jarred/git/DalESelfEBot/ur3_localisation/config/params.yaml";
    
    YAML::Node config = YAML::LoadFile(yaml_path);
    if (!config["corner_positions"]) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load corner_positions from params.yaml!");
        return 0.05;  // Default canvas height if loading fails
    }

    double total_z = 0.0;
    int count = 0;
    for (const auto& corner : config["corner_positions"]) {
        total_z += corner["z"].as<double>();
        count++;
    }

    return (count > 0) ? (total_z / count) : 0.05;  // Default to 50mm if something goes wrong
}
