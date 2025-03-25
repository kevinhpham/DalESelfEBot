// #include "spline_follower.hpp"
// #include <rclcpp/rclcpp.hpp>

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<SplineFollower>();
//     // rclcpp::executors::SingleThreadedExecutor executor;
//     // executor.add_node(node);
//     // executor.spin();
//     node->on_activate();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

#include <memory>
#include <thread>

#include "spline_follower.hpp"
#include <rclcpp/rclcpp.hpp>

void load_parameters_from_yaml(std::shared_ptr<rclcpp::Node> node, const std::string &yaml_file)
{
    YAML::Node yaml = YAML::LoadFile(yaml_file);
    for (auto it = yaml.begin(); it != yaml.end(); ++it)
    {
        const auto &param_name = it->first.as<std::string>();
        const auto &param_value = it->second;

        if (param_value.IsScalar())
        {
            node->set_parameter(rclcpp::Parameter(param_name, param_value.as<std::string>()));
        }
        else if (param_value.IsSequence())
        {
            std::vector<double> values;
            for (const auto &val : param_value)
            {
                values.push_back(val.as<double>());
            }
            node->set_parameter(rclcpp::Parameter(param_name, values));
        }
        // Add more cases if you expect more complex structures
    }
}

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);

//     auto node_one = std::make_shared<rclcpp::Node>("hello_moveit");

//     // Point this to your kinematics.yaml
//     std::string yaml_file = "/home/jarred/git/DalESelfEBot/ur3_control/config/kinematics.yaml";
//     load_parameters_from_yaml(node_one, yaml_file);

//     auto node = std::make_shared<SplineFollower>();
//     // node->on_activate();  // Starts the state machine

//     // rclcpp::spin(node);   // Keep spinning while the state machine runs

//     // Spin up a SingleThreadedExecutor for the current state monitor
//     rclcpp::executors::SingleThreadedExecutor executor;
//     executor.add_node(node);
//     executor.add_node(node_one);
//     auto spinner = std::thread([&executor]() { executor.spin(); });

//     node->on_activate();  // Starts the state machine

//     rclcpp::shutdown();

//     spinner.join();
//     return 0;
// }

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

    // // Get the current end-effector pose
    // auto const end_effector_pose = move_group_interface.getCurrentPose();
    // // Log the current end-effector pose
    // RCLCPP_INFO(logger, "End-effector pose: %f %f %f",
    //     end_effector_pose.pose.position.x,
    //     end_effector_pose.pose.position.y,
    //     end_effector_pose.pose.position.z
    // );

    auto control = std::make_shared<SplineFollower>();
    // control->on_activate();  // Starts the state machine

    // control->setSafeStartPose();

    // // geometry_msgs::msg::Pose new_pose;
    // // new_pose.position.x = 0.25;
    // // new_pose.position.y = 0.25;
    // // new_pose.position.z = 0.25;

    // std::vector<geometry_msgs::msg::Pose> path_to_safe_start = 
    // control->computeLinearInterpolationPath(end_effector_pose.pose, control->safe_start_pose_, 10);

    // moveit_msgs::msg::RobotTrajectory trajectory;
    // // double fraction = move_group_interface.computeCartesianPath(path_to_safe_start, 0.00001, 0.0, trajectory);

    // move_group_interface.setMaxVelocityScalingFactor(1.0);
    // move_group_interface.setGoalTolerance(0.001);

    // const double jump_threshold = 0.0;
    // const double eef_step = 1.0;
    // double fraction = move_group_interface.computeCartesianPath(path_to_safe_start, eef_step, jump_threshold, trajectory);
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
    // my_plan2.trajectory_ = trajectory;
    // move_group_interface.execute(my_plan2);

    // if (fraction < 0.95) { 
    //     RCLCPP_ERROR(logger, "Only achieved %.2f%% of the path", fraction * 100);
    // }

    // moveit::planning_interface::MoveGroupInterface::Plan plan;
    // plan.trajectory_ = trajectory;
    // move_group_interface.execute(plan);

    control->loadSplines();

    while (rclcpp::ok()) {
        switch (control->state_) {
            case SplineFollower::State::INIT:
            {
                // auto const end_effector_pose = move_group_interface.getCurrentPose();
                // Set the safe start pose based on localization data
                control->setSafeStartPose();

                move_group_interface.setPoseTarget(control->safe_start_pose_);
                move_group_interface.move();

                // const std::vector<geometry_msgs::msg::Pose> path_to_safe_start = 
                // control->computeLinearInterpolationPath(end_effector_pose.pose, control->safe_start_pose_, 10);

                

                // move_group_interface.setMaxVelocityScalingFactor(1.0);
                // move_group_interface.setGoalTolerance(0.005);

                // moveit_msgs::msg::RobotTrajectory trajectory;

                // const double jump_threshold = 0.0;
                // const double eef_step = 1.0;

                // double fraction;

                // while(fraction < 0.95){
                //     fraction = move_group_interface.computeCartesianPath(path_to_safe_start, eef_step, jump_threshold, trajectory);
                // }
                // double fraction = move_group_interface.computeCartesianPath(path_to_safe_start, eef_step, jump_threshold, trajectory);
                // moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
                // my_plan2.trajectory_ = trajectory;
                // move_group_interface.execute(my_plan2);

                // if (fraction < 0.95) { 
                //     RCLCPP_ERROR(logger, "Only achieved %.2f%% of the path", fraction * 100);
                // }

                control->state_ = SplineFollower::State::MOVE_TO_INTERMEDIATE_POS;
                break;

            }

            case SplineFollower::State::MOVE_TO_INTERMEDIATE_POS:
            {
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

                    // moveToPose(intermediate_pose_);

                    // // ✅ Get the current pose of the robot
                    // geometry_msgs::msg::Pose current_pose = getCurrentRobotPose();

                    // ✅ Compute a straight-line interpolated path to the intermediate poses
                    // std::vector<geometry_msgs::msg::Pose> path_to_intermediate =
                    //     computeLinearInterpolationPath(current_pose, intermediate_pose, 100);

                    // // ✅ Execute the trajectory to move to the intermediate pose
                    // executeTrajectory(path_to_intermediate);

                    // ✅ Move to the next state once the move is complete
                    control->state_ = SplineFollower::State::MOVE_TO_CANVAS;
                } 
                else {
                    // ✅ If no more splines left, go to STOP state
                    control->state_ = SplineFollower::State::STOP;
                }
                break;
            }

            case SplineFollower::State::MOVE_TO_CANVAS:
            {
                // // ✅ Calculate the average Z position of the canvas
                double canvas_z = control->calculateAverageCanvasHeight();

                // // // ✅ Get the current pose of the robot
                // // geometry_msgs::msg::Pose current_pose = getCurrentRobotPose();

                // // ✅ Set the target pose (same x, y as current pose, but at canvas height)
                control->canvas_pose_ = control->intermediate_pose_;
                control->canvas_pose_.position.z = canvas_z;  // Move to the canvas surface

                move_group_interface.setPoseTarget(control->canvas_pose_);
                move_group_interface.move();


                // moveToPose(canvas_pose_);

                // // // ✅ Compute a straight-line interpolated path to the canvas
                // // std::vector<geometry_msgs::msg::Pose> path_to_canvas =
                // //     computeLinearInterpolationPath(current_pose, canvas_pose, 10);

                // // // ✅ Execute the trajectory to move to the canvas
                // // executeTrajectory(path_to_canvas);

                // // ✅ Move to next state to start drawing
                control->state_ = SplineFollower::State::MOVE_THROUGH_DRAWING_TRAJECTORY;
                break;
            }

            case SplineFollower::State::MOVE_THROUGH_DRAWING_TRAJECTORY:
                // // generateDrawingTrajectory();
                // if (executeTrajectory(current_trajectory_)) {
                //     current_spline_index_++;
                //     state_ = State::MOVE_OFF_CANVAS;
                // } else {
                //     RCLCPP_ERROR(this->get_logger(), "Failed to execute drawing trajectory.");
                //     state_ = State::STOP;
                // }
                // std::cout << "Reached here" << std::endl;
                // generateDrawingTrajectory();
                // move_group_->execute(drawing_trajectory_);
                // current_spline_index_++;
                // state_ = State::MOVE_OFF_CANVAS;
                break;

            case SplineFollower::State::MOVE_OFF_CANVAS:
            {
                // // // ✅ Get the current pose of the robot
                // // geometry_msgs::msg::Pose current_pose = getCurrentRobotPose();
            
                // // ✅ Get the canvas height
                // double canvas_z = calculateAverageCanvasHeight();
            
                // // ✅ Compute the safe lifted pose (100mm above the canvas)
                // geometry_msgs::msg::Pose lifted_pose = canvas_pose_;
                // lifted_pose.position.z = canvas_z + 0.1;  // Move 100mm up

                // moveToPose(canvas_pose_);
            
                // // // ✅ Compute a straight-line interpolated trajectory
                // // std::vector<geometry_msgs::msg::Pose> path_off_canvas =
                // //     computeLinearInterpolationPath(current_pose, lifted_pose, 10);
            
                // // ✅ Execute the trajectory to lift off the canvas
                // // executeTrajectory(path_off_canvas);
            
                // // ✅ Move to the next state (going to the next spline or stopping)
                // current_spline_index_++;
                // if (current_spline_index_ < spline_data_["splines"].size()) {
                //     state_ = State::MOVE_TO_INTERMEDIATE_POS;
                // } else {
                //     state_ = State::STOP;
                // }
            
                break;
            }                

            case SplineFollower::State::STOP:
                // moveToSafeEndPose();
                // stopExecution();
                // RCLCPP_INFO(logger, "All splines completed. Stopping MoveIt and shutting down...");
                return 0;
        }
    }

    // Shutdown ROS
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
