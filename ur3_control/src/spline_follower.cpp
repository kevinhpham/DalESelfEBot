#include "spline_follower.hpp"
#include <fstream>
#include <thread>

using namespace std::chrono_literals;
using json = nlohmann::json;

SplineFollower::SplineFollower() : Node("spline_follower"), state_(State::INIT), current_spline_index_(0), tf_buffer_(this->get_clock()),
tf_listener_(tf_buffer_){
    RCLCPP_INFO(this->get_logger(), "SplineFollower node created.");
}

void SplineFollower::on_activate() {
    auto node_shared_ptr = shared_from_this();
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node_shared_ptr, "ur_manipulator");

    // move_group_->startStateMonitor();  // Start state monitoring service
    rclcpp::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized.");
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    move_group_->setMaxVelocityScalingFactor(1.0);
    move_group_->setMaxAccelerationScalingFactor(1.0);

    addGroundPlane();

    if (!loadSplines()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load splines. Exiting...");
        return;
    }

    move_group_->setPoseReferenceFrame("base_link");
    RCLCPP_INFO(this->get_logger(), "Pose reference frame: %s", move_group_->getPoseReferenceFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());


    // runStateMachine();
    geometry_msgs::msg::Pose current_pose = getCurrentRobotPose();

    // geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();

    // std::cout << "Current Pose: x = " << current_pose.pose.position.x << " y = " << current_pose.pose.position.x 
    // << " z = " << current_pose.pose.position.x << std::endl;

    geometry_msgs::msg::Pose new_pose;
    new_pose.position.x = 0.1;
    new_pose.position.y = 0.1;
    new_pose.position.z = 0.1;

    std::vector<geometry_msgs::msg::Pose> path = 
                    computeLinearInterpolationPath(current_pose, new_pose, 10);

    executeTrajectory(path);

    current_pose = getCurrentRobotPose();

    std::cout << "Current Pose: x = " << current_pose.position.x << " y = " << current_pose.position.y 
    << " z = " << current_pose.position.z << std::endl;


}

void SplineFollower::runStateMachine() {
    while (rclcpp::ok()) {
        switch (state_) {
            case State::INIT:
            {
                // Set the safe start pose based on localization data
                setSafeStartPose();

                std::cout << "Safe Start Pose: x = " << safe_start_pose_.position.x << " y = " << safe_start_pose_.position.y << " z = "
                << safe_start_pose_.position.z << std::endl;

                // ✅ Get the current pose of the robot
                geometry_msgs::msg::Pose current_pose = getCurrentRobotPose();

                std::cout << "Current Pose: x = " << current_pose.position.x << " y = " << current_pose.position.y 
                << " z = " << current_pose.position.z << std::endl;

                geometry_msgs::msg::Pose next_pose = safe_start_pose_;

                // ✅ Compute a straight-line interpolated path to the safe start pose
                std::vector<geometry_msgs::msg::Pose> path_to_safe_start = 
                    computeLinearInterpolationPath(current_pose, next_pose, 10);

                // ✅ Execute the trajectory to move safely to the start pose
                executeTrajectory(path_to_safe_start);

                // ✅ Once the move is complete, transition to the next state
                state_ = State::MOVE_TO_INTERMEDIATE_POS;
                break;
            }

            case State::MOVE_TO_INTERMEDIATE_POS:
            {
                // ✅ Ensure we have a next spline to move to
                if (current_spline_index_ < spline_data_["splines"].size()) {
                    
                    // ✅ Get the first waypoint of the next spline
                    json next_spline = spline_data_["splines"][current_spline_index_];
                    json first_waypoint = next_spline["waypoints"][0];

                    // ✅ Calculate the intermediate pose (100mm above the first waypoint)
                    geometry_msgs::msg::Pose intermediate_pose;
                    intermediate_pose.position.x = first_waypoint[0].get<double>();
                    intermediate_pose.position.y = first_waypoint[1].get<double>();
                    intermediate_pose.position.z = first_waypoint[2].get<double>() + 0.1;  // 100mm above
                    
                    // ✅ Ensure the orientation is pointing straight down
                    intermediate_pose.orientation.x = 0.0;
                    intermediate_pose.orientation.y = 1.0;
                    intermediate_pose.orientation.z = 0.0;
                    intermediate_pose.orientation.w = 0.0;

                    // ✅ Get the current pose of the robot
                    geometry_msgs::msg::Pose current_pose = getCurrentRobotPose();

                    // ✅ Compute a straight-line interpolated path to the intermediate poses
                    std::vector<geometry_msgs::msg::Pose> path_to_intermediate =
                        computeLinearInterpolationPath(current_pose, intermediate_pose, 100);

                    // ✅ Execute the trajectory to move to the intermediate pose
                    executeTrajectory(path_to_intermediate);

                    // ✅ Move to the next state once the move is complete
                    state_ = State::MOVE_TO_CANVAS;
                } 
                else {
                    // ✅ If no more splines left, go to STOP state
                    state_ = State::STOP;
                }
                break;
            }

            case State::MOVE_TO_CANVAS:
            {
                // ✅ Calculate the average Z position of the canvas
                double canvas_z = calculateAverageCanvasHeight();

                // ✅ Get the current pose of the robot
                geometry_msgs::msg::Pose current_pose = getCurrentRobotPose();

                // ✅ Set the target pose (same x, y as current pose, but at canvas height)
                geometry_msgs::msg::Pose canvas_pose = current_pose;
                canvas_pose.position.z = canvas_z;  // Move to the canvas surface

                // ✅ Compute a straight-line interpolated path to the canvas
                std::vector<geometry_msgs::msg::Pose> path_to_canvas =
                    computeLinearInterpolationPath(current_pose, canvas_pose, 10);

                // ✅ Execute the trajectory to move to the canvas
                executeTrajectory(path_to_canvas);

                // ✅ Move to next state to start drawing
                state_ = State::MOVE_THROUGH_DRAWING_TRAJECTORY;
                break;
            }

            case State::MOVE_THROUGH_DRAWING_TRAJECTORY:
                if (executeTrajectory(current_trajectory_)) {
                    current_spline_index_++;
                    state_ = State::MOVE_OFF_CANVAS;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to execute drawing trajectory.");
                    state_ = State::STOP;
                }
                break;

            case State::MOVE_OFF_CANVAS:
            {
                // ✅ Get the current pose of the robot
                geometry_msgs::msg::Pose current_pose = getCurrentRobotPose();
            
                // ✅ Get the canvas height
                double canvas_z = calculateAverageCanvasHeight();
            
                // ✅ Compute the safe lifted pose (100mm above the canvas)
                geometry_msgs::msg::Pose lifted_pose = current_pose;
                lifted_pose.position.z = canvas_z + 0.1;  // Move 100mm up
            
                // ✅ Compute a straight-line interpolated trajectory
                std::vector<geometry_msgs::msg::Pose> path_off_canvas =
                    computeLinearInterpolationPath(current_pose, lifted_pose, 10);
            
                // ✅ Execute the trajectory to lift off the canvas
                executeTrajectory(path_off_canvas);
            
                // ✅ Move to the next state (going to the next spline or stopping)
                current_spline_index_++;
                if (current_spline_index_ < spline_data_["splines"].size()) {
                    state_ = State::MOVE_TO_INTERMEDIATE_POS;
                } else {
                    state_ = State::STOP;
                }
            
                break;
            }                

            case State::STOP:
                moveToSafeEndPose();
                stopExecution();
                RCLCPP_INFO(this->get_logger(), "All splines completed. Stopping MoveIt and shutting down...");
                // rclcpp::shutdown();
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

bool SplineFollower::executeTrajectory(const std::vector<geometry_msgs::msg::Pose>& waypoints) {
    if (waypoints.empty()) return false;

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_->computeCartesianPath(waypoints, 0.00001, 0.0, trajectory);
    // std::cout << "Made it here." << std::endl;
    // if (fraction < 0.95) return false;

    // std::cout << "Made it here." << std::endl;

    if (fraction < 0.95) { 
        RCLCPP_ERROR(this->get_logger(), "Only achieved %.2f%% of the path", fraction * 100);
    }


    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    move_group_->execute(plan);

    std::this_thread::sleep_for(5s);
    // std::cout << "Made it here." << std::endl;
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
    // ✅ Get the current pose of the robot
    geometry_msgs::msg::Pose current_pose = getCurrentRobotPose();

    // ✅ Safe end pose is the same as the safe start pose
    geometry_msgs::msg::Pose safe_end_pose = safe_start_pose_;

    // ✅ Compute a straight-line interpolated trajectory
    std::vector<geometry_msgs::msg::Pose> path_to_safe_end =
        computeLinearInterpolationPath(current_pose, safe_end_pose, 10);

    // ✅ Execute the trajectory to move safely to the end position
    executeTrajectory(path_to_safe_end);

    // ✅ Stop MoveIt execution after reaching the safe pose
    RCLCPP_INFO(this->get_logger(), "Robot safely moved to end position. Shutting down MoveIt execution.");
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

std::vector<geometry_msgs::msg::Pose> SplineFollower::computeLinearInterpolationPath(
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

        // ✅ Check reachability
        bool reachable = move_group_->setApproximateJointValueTarget(interpolated_pose);
        if (reachable) {
            RCLCPP_INFO(this->get_logger(), "Waypoint %d reachable at (x=%.3f, y=%.3f, z=%.3f)", 
                        i, interpolated_pose.position.x, interpolated_pose.position.y, interpolated_pose.position.z);
        } else {
            RCLCPP_WARN(this->get_logger(), "Waypoint %d NOT reachable at (x=%.3f, y=%.3f, z=%.3f)", 
                        i, interpolated_pose.position.x, interpolated_pose.position.y, interpolated_pose.position.z);
        }

        waypoints.push_back(interpolated_pose);
    }

    std::cout << "Calculated Linear Waypoints." << std::endl;

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

geometry_msgs::msg::Pose SplineFollower::getCurrentRobotPose() {
    geometry_msgs::msg::Pose pose;
    try {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped = tf_buffer_.lookupTransform("base_link", "tool0", tf2::TimePointZero);

        pose.position.x = transform_stamped.transform.translation.x;
        pose.position.y = transform_stamped.transform.translation.y;
        pose.position.z = transform_stamped.transform.translation.z;

        pose.orientation.x = transform_stamped.transform.rotation.x;
        pose.orientation.y = transform_stamped.transform.rotation.y;
        pose.orientation.z = transform_stamped.transform.rotation.z;
        pose.orientation.w = transform_stamped.transform.rotation.w;

    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Could not get current robot pose: %s", ex.what());
    }
    return pose;
}