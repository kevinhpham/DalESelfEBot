#include "spline_follower.hpp"
#include <fstream>
#include <thread>

using namespace std::chrono_literals;
using json = nlohmann::json;

SplineFollower::SplineFollower() : Node("spline_follower"), state_(State::INIT), current_spline_index_(0) {
    RCLCPP_INFO(this->get_logger(), "SplineFollower node created.");
}

void SplineFollower::on_activate() {
    auto node_shared_ptr = shared_from_this();
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node_shared_ptr, "ur_manipulator");

    RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized.");
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    move_group_->setMaxVelocityScalingFactor(1.0);
    move_group_->setMaxAccelerationScalingFactor(1.0);

    addGroundPlane();

    if (!loadSplines()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load splines. Exiting...");
        return;
    }

    if (!initializeSafePoses()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize safe poses. Exiting...");
        return;
    }

    runStateMachine();
}

void SplineFollower::runStateMachine() {
    while (rclcpp::ok()) {
        switch (state_) {
            case State::INIT:
                setSafeStartPose();
                state_ = State::GEN_INTERMEDIATE_TRAJ;
                break;

            case State::GEN_INTERMEDIATE_TRAJ:
                if (current_spline_index_ < spline_data_["splines"].size()) {
                    generateIntermediateTrajectory();
                    state_ = State::MOVE_TO_INTERMEDIATE_POS;
                } else {
                    state_ = State::STOP;
                }
                break;

            case State::MOVE_TO_INTERMEDIATE_POS:
                if (executeTrajectory(current_trajectory_)) {
                    state_ = State::GEN_DRAWING_TRAJECTORY;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to move to intermediate position.");
                    state_ = State::STOP;
                }
                break;

            case State::GEN_DRAWING_TRAJECTORY:
                generateDrawingTrajectory();
                state_ = State::MOVE_THROUGH_DRAWING_TRAJECTORY;
                break;

            case State::MOVE_THROUGH_DRAWING_TRAJECTORY:
                if (executeTrajectory(current_trajectory_)) {
                    current_spline_index_++;
                    state_ = State::GEN_INTERMEDIATE_TRAJ;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to execute drawing trajectory.");
                    state_ = State::STOP;
                }
                break;

            case State::STOP:
                moveToSafeEndPose();
                stopExecution();
                RCLCPP_INFO(this->get_logger(), "All splines completed. Stopping MoveIt and shutting down...");
                rclcpp::shutdown();
                return;
        }
    }
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
    ground_pose.position.z = -0.005;

    ground.primitives.push_back(ground_shape);
    ground.primitive_poses.push_back(ground_pose);
    ground.operation = ground.ADD;

    planning_scene_interface.applyCollisionObject(ground);
}

bool SplineFollower::loadSplines() {
    std::ifstream file("/home/jarred/git/DalESelfEBot/ur3_control/config/drawing_path.json");
    if (!file) {
        RCLCPP_ERROR(this->get_logger(), "Could not open spline JSON file.");
        return false;
    }

    file >> spline_data_;
    return !spline_data_["splines"].empty();
}

bool SplineFollower::initializeSafePoses() {
    std::string localization_params_path = "/path/to/localization/params.yaml";
    std::ifstream file(localization_params_path);
    if (!file) return false;

    YAML::Node config = YAML::Load(file);
    if (!config["corner_positions"] || config["corner_positions"].size() < 4) return false;

    safe_start_pose_ = parsePose(config["corner_positions"][0]);
    safe_end_pose_ = parsePose(config["corner_positions"][2]);
    return true;
}

geometry_msgs::msg::Pose SplineFollower::parsePose(const YAML::Node& node) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = node["x"].as<double>();
    pose.position.y = node["y"].as<double>();
    pose.position.z = node["z"].as<double>();
    pose.orientation.x = node["qx"].as<double>();
    pose.orientation.y = node["qy"].as<double>();
    pose.orientation.z = node["qz"].as<double>();
    pose.orientation.w = node["qw"].as<double>();
    return pose;
}

void SplineFollower::setSafeStartPose() {
    executePoseTarget(safe_start_pose_, "Moving to safe start pose");
}

bool SplineFollower::executeTrajectory(const std::vector<geometry_msgs::msg::Pose>& waypoints) {
    if (waypoints.empty()) return false;

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    if (fraction < 0.95) return false;

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    move_group_->execute(plan);

    std::this_thread::sleep_for(1s);
    return true;
}

void SplineFollower::stopExecution() {
    move_group_->stop();
    move_group_->clearPoseTargets();
    std::this_thread::sleep_for(1s);
}

void SplineFollower::generateIntermediateTrajectory() {
    if (remaining_splines_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No remaining splines. Skipping intermediate trajectory generation.");
        return;
    }

    // Get the next spline's first waypoint
    geometry_msgs::msg::Pose next_waypoint = getWaypointPose(remaining_splines_.front()["waypoints"][0], 0.05);

    // Generate a straight-line trajectory from the current pose to the next waypoint (lifted position)
    geometry_msgs::msg::Pose current_pose = move_group_->getCurrentPose().pose;
    std::vector<geometry_msgs::msg::Pose> waypoints = {current_pose, next_waypoint};

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (fraction < 0.95) {
        RCLCPP_ERROR(this->get_logger(), "Failed to compute intermediate trajectory (%.2f%%).", fraction * 100);
        return;
    }

    intermediate_trajectory_ = trajectory;
    RCLCPP_INFO(this->get_logger(), "Intermediate trajectory generated successfully.");
}

void SplineFollower::generateDrawingTrajectory() {
    if (remaining_splines_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No remaining splines to generate drawing trajectory.");
        return;
    }

    std::vector<geometry_msgs::msg::Pose> waypoints;
    for (const auto& waypoint : remaining_splines_.front()["waypoints"]) {
        waypoints.push_back(getWaypointPose(waypoint, 0.0));  // No lift, just drawing
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (fraction < 0.95) {
        RCLCPP_ERROR(this->get_logger(), "Failed to compute drawing trajectory (%.2f%%).", fraction * 100);
        return;
    }

    drawing_trajectory_ = trajectory;
    RCLCPP_INFO(this->get_logger(), "Drawing trajectory generated successfully.");
}

void SplineFollower::executePoseTarget(const geometry_msgs::msg::Pose& target_pose, const std::string& description) {
    move_group_->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(this->get_logger(), "%s...", description.c_str());
        move_group_->execute(plan);

        // ✅ Ensures MoveIt has time to execute before moving on
        rclcpp::sleep_for(std::chrono::seconds(5));

        RCLCPP_INFO(this->get_logger(), "%s completed.", description.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to plan: %s", description.c_str());
    }
}

void SplineFollower::moveToSafeEndPose() {
    // Move back to the originally determined safe starting position
    geometry_msgs::msg::Pose safe_end_pose = getWaypointPose(remaining_splines_.front()["waypoints"][0], 0.05);
    
    executePoseTarget(safe_end_pose, "Moving back to safe end pose");

    // Stop MoveIt execution after reaching the safe pose
    RCLCPP_INFO(this->get_logger(), "Shutting down MoveIt execution.");
}

geometry_msgs::msg::Pose SplineFollower::getWaypointPose(const json& waypoint, double lift) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = waypoint[0].get<double>();
    pose.position.y = waypoint[1].get<double>();
    pose.position.z = waypoint[2].get<double>() + lift;

    pose.orientation.x = 0.0;
    pose.orientation.y = 1.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;

    return pose;
}
