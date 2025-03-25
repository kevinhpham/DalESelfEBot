#include <memory>
#include <thread>

#include "spline_follower.hpp"
#include <rclcpp/rclcpp.hpp>


int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("moveit");

    // Spin up a SingleThreadedExecutor for the current state monitor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

    move_group_interface.setMaxVelocityScalingFactor(1.0);  // 1.0 = 100% of max velocity
    move_group_interface.setMaxAccelerationScalingFactor(1.0); // 1.0 = 100% of max acceleration
    // move_group_interface.setPlannerId("PTP");

    auto control = std::make_shared<SplineFollower>();

    control->loadSplines();

    control->addGroundPlane();

    while (rclcpp::ok()) {
        switch (control->state_) {
            case SplineFollower::State::INIT:
            {
                RCLCPP_INFO(logger, "State: Init");
                // Set the safe start pose based on localization data
                control->setSafeStartPose();

                move_group_interface.setPoseTarget(control->safe_start_pose_);
                move_group_interface.move();
                RCLCPP_INFO(logger, "Moved to safe start pose");

                control->state_ = SplineFollower::State::MOVE_TO_INTERMEDIATE_POS;
                break;

            }

            case SplineFollower::State::MOVE_TO_INTERMEDIATE_POS:
            {

                RCLCPP_INFO(logger, "State: Move to intermediate pos.");

                // Ensure we have a next spline to move to
                if (control->current_spline_index_ < control->spline_data_["splines"].size()) {
                    
                    // Get the first waypoint of the next spline
                    json next_spline = control->spline_data_["splines"][control->current_spline_index_];
                    json first_waypoint = next_spline["waypoints"][0];

                    // Calculate the intermediate pose (100mm above the first waypoint)
                    // geometry_msgs::msg::Pose intermediate_pose;
                    control->intermediate_pose_.position.x = first_waypoint[0].get<double>();
                    control->intermediate_pose_.position.y = first_waypoint[1].get<double>();
                    control->intermediate_pose_.position.z = first_waypoint[2].get<double>() + 0.1;  // 100mm above
                    
                    // Ensure the orientation is pointing straight down
                    control->intermediate_pose_.orientation.x = 0.0;
                    control->intermediate_pose_.orientation.y = 1.0;
                    control->intermediate_pose_.orientation.z = 0.0;
                    control->intermediate_pose_.orientation.w = 0.0;

                    move_group_interface.setPoseTarget(control->intermediate_pose_);
                    move_group_interface.move();
                    control->state_ = SplineFollower::State::MOVE_TO_CANVAS;
                    RCLCPP_INFO(logger, "Moved to intermediate pose.");
                } 
                else {
                    // If no more splines left, go to STOP state
                    control->state_ = SplineFollower::State::STOP;
                }
                break;
            }

            case SplineFollower::State::MOVE_TO_CANVAS:
            {
                RCLCPP_INFO(logger, "State: Move to canvas.");
                // // ✅ Calculate the average Z position of the canvas
                double canvas_z = control->calculateAverageCanvasHeight();

                control->canvas_pose_ = control->intermediate_pose_;
                control->canvas_pose_.position.z = canvas_z;  // Move to the canvas surface

                move_group_interface.setPoseTarget(control->canvas_pose_);
                move_group_interface.move();

                RCLCPP_INFO(logger, "Moved to canvas.");

                control->state_ = SplineFollower::State::MOVE_THROUGH_DRAWING_TRAJECTORY;
                break;
            }

            case SplineFollower::State::MOVE_THROUGH_DRAWING_TRAJECTORY:
            {
                RCLCPP_INFO(logger, "State: Move through drawing trajectory.");
                std::vector<geometry_msgs::msg::Pose> waypoints;
                for (const auto& waypoint : control->spline_data_["splines"].at(control->current_spline_index_)["waypoints"]) {
                    geometry_msgs::msg::Pose pose;
                    pose.position.x = waypoint[0].get<double>();
                    pose.position.y = waypoint[1].get<double>();
                    pose.position.z = waypoint[2].get<double>();  // you can add lift here if needed
            
                    pose.orientation.x = 0.0;
                    pose.orientation.y = 1.0;
                    pose.orientation.z = 0.0;
                    pose.orientation.w = 0.0;
            
                    waypoints.push_back(pose);
                }
            
                moveit_msgs::msg::RobotTrajectory trajectory;
                double fraction = move_group_interface.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
            
                std::cout << "Acheieved " << fraction << " of the path " << std::endl;

                RCLCPP_INFO(logger, "Achieved: %ld", control->current_spline_index_*100, " percent of the path.");
            
                if (fraction < 0.95) {
                    RCLCPP_ERROR(logger, "Failed to compute drawing trajectory (%.2f%%).", fraction * 100);
                }
                RCLCPP_INFO(logger, "Completed spline: %ld", control->current_spline_index_);

                control->state_ = SplineFollower::State::MOVE_OFF_CANVAS;

                break;
            }

            case SplineFollower::State::MOVE_OFF_CANVAS:
            {
                RCLCPP_INFO(logger, "State: Move off canvas.");
                // Get the canvas height
                double canvas_z = control->calculateAverageCanvasHeight();
            
                // Compute the safe lifted pose (100mm above the canvas)
                geometry_msgs::msg::Pose lifted_pose = control->canvas_pose_;
                lifted_pose.position.z = canvas_z + 0.1;  // Move 100mm up

                move_group_interface.setPoseTarget(lifted_pose);
                move_group_interface.move();

            
                // Move to the next state (going to the next spline or stopping)
                control->current_spline_index_++;
                if (control->current_spline_index_ < control->spline_data_["splines"].size()) {
                    RCLCPP_INFO(logger, "Moved off canvas.");
                    control->state_ = SplineFollower::State::MOVE_TO_INTERMEDIATE_POS;
                } else {
                    control->state_ = SplineFollower::State::STOP;
                }
            
                break;
            }                

            case SplineFollower::State::STOP:
                RCLCPP_INFO(logger, "State: Stop.");
                RCLCPP_INFO(logger, "All splines completed.");
                return 0;
        }
    }

    // Shutdown ROS
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
