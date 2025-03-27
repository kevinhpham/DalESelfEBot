#include <memory>
#include <thread>

#include "spline_follower.hpp"
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>


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

    move_group_interface.setMaxVelocityScalingFactor(0.1);  // 1.0 = 100% of max velocity
    move_group_interface.setMaxAccelerationScalingFactor(1.0); // 1.0 = 100% of max acceleration
    // move_group_interface.setPlannerId("PTP");
    // move_group_interface.startStateMonitor(1.0);

    auto control = std::make_shared<SplineFollower>();

    control->loadSplines();

    control->addGroundPlane();

    while (rclcpp::ok()) {
        switch (control->state_) {
            case SplineFollower::State::INIT:
            {
                RCLCPP_INFO(logger, "State: Init");

                control->setSafeStartPose();

                // geometry_msgs::msg::Pose current_pose = move_group_interface.getCurrentPose().pose;

                // std::cout << "Current Pose is x = " << current_pose.position.x << " y = " 
                // << current_pose.position.y << " z = " << current_pose.position.z << std::endl;

                // geometry_msgs::msg::Pose goal_pose = control->safe_start_pose_;

                // std::vector<geometry_msgs::msg::Pose> waypoints = 
                // control->computeLinearInterpolationPath(current_pose, goal_pose, 10);

                // std::vector<geometry_msgs::msg::Pose> reachable_waypoints;

                // for (auto& waypoint : waypoints) {
                //     std::vector<geometry_msgs::msg::Pose> segment;
                //     segment.push_back(waypoint);

                //     moveit_msgs::msg::RobotTrajectory trajectory;
                //     const double eef_step = 0.01;  // e.g., 1 cm resolution
                //     const double jump_threshold = 0.0;  // disable jump threshold
                //     double fraction = move_group_interface.computeCartesianPath(segment, eef_step, jump_threshold, trajectory);

                //     if (fraction == 1.0) {
                //         std::cout << "Waypoint is reachable via straight-line Cartesian path" << std::endl;

                //         // Optionally execute
                //         moveit::planning_interface::MoveGroupInterface::Plan plan;
                //         plan.trajectory_ = trajectory;
                //         move_group_interface.execute(plan);

                //         reachable_waypoints.push_back(waypoint);
                //     } else {
                //         std::cout << "Waypoint is NOT reachable via straight-line Cartesian path" << std::endl;
                //     }
                // }

                // std::cout << "Waypoints in total = " << waypoints.size() << std::endl;

                // geometry_msgs::msg::Pose waypoint;
                // waypoint = control->safe_start_pose_;

                // std::vector<geometry_msgs::msg::Pose> waypoints;
                // waypoints.push_back(safe_start_);

                // const double jump_threshold = 0.0;
                // const double eef_step = 0.01;

                // for(auto waypoint : waypoints){
                //     move_group_interface.setPoseTarget(waypoint);
                //     move_group_interface.move();
                // }

                // for(auto waypoint : waypoints){
                //     std::cout << "Waypoint: x = " << waypoint.position.x << " y = " << waypoint.position.y << " z = " 
                //     << waypoint.position.z << "." << std::endl;
                // }

                // int count = 0;

                // for (auto waypoint : waypoints) {
                //     move_group_interface.setPoseTarget(waypoint);

                //     moveit::planning_interface::MoveGroupInterface::Plan plan;
                //     auto result = move_group_interface.plan(plan);
                //     move_group_interface.execute(plan);

                //     if (result == moveit::core::MoveItErrorCode::SUCCESS) {
                //         std::cout << "Waypoint " << count << " is reachable" << std::endl;
                //     } else {
                //         std::cout << "Waypoint " << count << " is unreachable" << std::endl;
                //         std::cout << "Erasing waypoint " << count << std::endl;
                //         waypoints.erase(waypoints.begin()+count);  // Remove unreachable waypoint
                //         count--;
                //     }
                //     count++;
                // }

                // std::cout << "Waypoints remaining = " << waypoints.size() << std::endl;

                // moveit_msgs::msg::RobotTrajectory trajectory;

                // double fraction = 0.0;
                // double eef_step = 0.01;
                // double jump_threshold = 0.0;
                // unsigned int it = 0;
                // int steps = 1;
                
                // while (fraction < 0.95 && it < 100) {
                //     fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
                //     std::cout << "Iteration: " << it << ", fraction: " << fraction 
                //               << ", eef_step: " << eef_step 
                //               << ", jump_threshold: " << jump_threshold << std::endl;
                    
                //     // eef_step += 0.005;  // Small increments are usually better
                //     // jump_threshold += 0.01;
                //     // it++;
                //     waypoints = control->computeLinearInterpolationPath(current_pose, goal_pose, steps);
                //     steps++;
                //     it--;
                
                //     rclcpp::sleep_for(std::chrono::milliseconds(50));  // Small pause
                // }

                // double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

                // std::cout << "Fraction of path computed = " << fraction*100 << " percent." << std::endl;
                
                
                // move_group_interface.move();
                

                double joint_target = -M_PI/4.0;
                moveit_msgs::msg::Constraints constraint;
                std::vector<moveit_msgs::msg::JointConstraint> joint_constraints;
                moveit_msgs::msg::JointConstraint shoulder_lift_joint_constraint;
                shoulder_lift_joint_constraint.set__joint_name("shoulder_lift_joint");
                shoulder_lift_joint_constraint.set__position(joint_target);

                shoulder_lift_joint_constraint.set__tolerance_above(M_PI/4.0);
                shoulder_lift_joint_constraint.set__tolerance_below(M_PI/4.0);
                shoulder_lift_joint_constraint.set__weight(1.0);

                joint_constraints.push_back(shoulder_lift_joint_constraint);
                constraint.set__joint_constraints(joint_constraints);

                moveit::planning_interface::MoveGroupInterface::Plan plan;
                move_group_interface.setPathConstraints(constraint);
                move_group_interface.setPoseTarget(control->safe_start_pose_);
                move_group_interface.setPlanningTime(10.0);
                move_group_interface.setPlannerId("PTP");
                
                move_group_interface.plan(plan);
                move_group_interface.execute(plan);

                move_group_interface.clearPathConstraints();
                
                RCLCPP_INFO(logger, "Moved to safe start pose");

                control->state_ = SplineFollower::State::MOVE_TO_INTERMEDIATE_POS;
                // control->state_ = SplineFollower::State::STOP;
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
                    control->intermediate_pose_.position.x = first_waypoint[0].get<double>();
                    control->intermediate_pose_.position.y = first_waypoint[1].get<double>();
                    control->intermediate_pose_.position.z = first_waypoint[2].get<double>() + 0.1;  // 100mm above
                    
                    // Ensure the orientation is pointing straight down
                    control->intermediate_pose_.orientation.x = 0.0;
                    control->intermediate_pose_.orientation.y = 1.0;
                    control->intermediate_pose_.orientation.z = 0.0;
                    control->intermediate_pose_.orientation.w = 0.0;

                    geometry_msgs::msg::Pose current_pose = move_group_interface.getCurrentPose().pose;
                    std::vector<geometry_msgs::msg::Pose> waypoints = 
                    control->computeLinearInterpolationPath(current_pose, control->intermediate_pose_, 10);
                    double jump_threshold = 0.0;
                    double eef_step = 0.01;
                    moveit_msgs::msg::RobotTrajectory trajectory;
                    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

                    if (fraction < 0.95) {
                        RCLCPP_ERROR(logger, "Failed to compute drawing trajectory (%.2f%%).", fraction * 100);
                    }

                    // move_group_interface.setPoseTarget(control->intermediate_pose_);
                    // move_group_interface.setPlannerId("LIN");
                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    plan.trajectory_ = trajectory;
                    // move_group_interface.plan(plan);
                    move_group_interface.execute(plan);
                    // move_group_interface.move();
                    control->state_ = SplineFollower::State::MOVE_TO_CANVAS;
                    // control->state_ = SplineFollower::State::STOP;
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
                // Calculate the average Z position of the canvas
                double canvas_z = control->calculateAverageCanvasHeight();

                control->canvas_pose_ = control->intermediate_pose_;
                control->canvas_pose_.position.z = canvas_z;  // Move to the canvas surface

                geometry_msgs::msg::Pose current_pose = move_group_interface.getCurrentPose().pose;
                std::vector<geometry_msgs::msg::Pose> waypoints = 
                control->computeLinearInterpolationPath(current_pose, control->canvas_pose_, 10);
                double jump_threshold = 0.0;
                double eef_step = 0.01;
                moveit_msgs::msg::RobotTrajectory trajectory;
                double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

                if (fraction < 0.95) {
                    RCLCPP_ERROR(logger, "Failed to compute drawing trajectory (%.2f%%).", fraction * 100);
                }

                moveit::planning_interface::MoveGroupInterface::Plan plan;
                plan.trajectory_ = trajectory;
                move_group_interface.execute(plan);

                // move_group_interface.setPoseTarget(control->canvas_pose_);
                // move_group_interface.move();

                RCLCPP_INFO(logger, "Moved to canvas.");

                control->state_ = SplineFollower::State::MOVE_THROUGH_DRAWING_TRAJECTORY;
                // control->state_ = SplineFollower::State::STOP;
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
            
                double jump_threshold = 0.0;
                double eef_step = 0.01;
                moveit_msgs::msg::RobotTrajectory trajectory;
                double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            
                // std::cout << "Acheieved " << fraction << " of the path " << std::endl;

                // RCLCPP_INFO(logger, "Achieved: %ld", control->current_spline_index_*100, " percent of the path.");
            
                if (fraction < 0.95) {
                    RCLCPP_ERROR(logger, "Failed to compute drawing trajectory (%.2f%%).", fraction * 100);
                }


                moveit::planning_interface::MoveGroupInterface::Plan plan;
                plan.trajectory_ = trajectory;
                move_group_interface.execute(plan);

                RCLCPP_INFO(logger, "Completed spline: %ld", control->current_spline_index_);

                // control->state_ = SplineFollower::State::MOVE_OFF_CANVAS;
                control->state_ = SplineFollower::State::STOP;

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
                rclcpp::shutdown();
                spinner.join();
                return 0;
        }
    }

    // Shutdown ROS
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
