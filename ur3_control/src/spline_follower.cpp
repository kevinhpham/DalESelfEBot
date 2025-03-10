#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Geometry>
#include <vector>
#include <fstream>
#include <nlohmann/json.hpp>
#include "trajectory_utils.hpp"
#include "ik_solver_kdl.hpp"  // Include the KDL IK solver

using namespace std;
using json = nlohmann::json;

class SplineFollower : public rclcpp::Node {
public:
    SplineFollower() : Node("spline_follower"), ik_solver_("/path/to/ur3e.urdf") {
        trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/scaled_joint_trajectory_controller/joint_trajectory", 10);
        loadSplines();
    }

    void startDrawing() {
        if (!ik_solver_.isInitialized()) {
            RCLCPP_ERROR(this->get_logger(), "IK Solver not initialized correctly!");
            return;
        }

        auto splines = spline_data_["splines"];
        if (splines.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No splines found in JSON!");
            return;
        }

        std::vector<double> first_waypoint = computeInverseKinematics(splines[0]["waypoints"][0].get<std::vector<double>>());

        moveToAboveStartPosition(first_waypoint);
        moveToFirstWaypoint(first_waypoint);

        for (size_t i = 0; i < splines.size() - 1; ++i) {
            executeSpline(splines[i]["waypoints"].get<std::vector<std::vector<double>>>());
            liftAndMove(splines[i]["waypoints"].back(), splines[i + 1]["waypoints"].front());
        }

        executeSpline(splines.back()["waypoints"].get<std::vector<std::vector<double>>>());
    }

private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
    json spline_data_;
    IKSolverKDL ik_solver_;  // Use KDL-based inverse kinematics solver

    void loadSplines() {
        ifstream file("config/drawing_path.json");
        file >> spline_data_;
    }

    std::vector<double> computeInverseKinematics(const std::vector<double>& cartesian_position) {
        std::vector<double> joint_positions = ik_solver_.computeInverseKinematics(cartesian_position);
        if (joint_positions.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute IK solution");
        }
        return joint_positions;
    }

    void moveToAboveStartPosition(const std::vector<double>& first_waypoint) {
        std::vector<double> above_position = first_waypoint;
        above_position[2] += 0.05;
        executeSpline({above_position});
    }

    void moveToFirstWaypoint(const std::vector<double>& first_waypoint) {
        executeSpline({first_waypoint});
    }

    void liftAndMove(const std::vector<double>& start_pos, const std::vector<double>& end_pos) {
        std::vector<double> up_position = start_pos;
        up_position[2] += 0.05;
        std::vector<double> down_position = end_pos;
        down_position[2] += 0.05;

        executeSpline({up_position, down_position});
        executeSpline({down_position, end_pos});
    }

    void executeSpline(const std::vector<std::vector<double>>& waypoints) {
        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

        double total_time = 5.0;
        double time_step = total_time / waypoints.size();

        for (size_t i = 0; i < waypoints.size(); ++i) {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = computeInverseKinematics(waypoints[i]);

            double t = time_step * (i + 1);
            builtin_interfaces::msg::Duration duration_msg;
            duration_msg.sec = static_cast<int>(t);
            duration_msg.nanosec = static_cast<int>((t - duration_msg.sec) * 1e9);
            point.time_from_start = duration_msg;

            traj_msg.points.push_back(point);
        }

        RCLCPP_INFO(this->get_logger(), "Executing spline trajectory...");
        trajectory_pub_->publish(traj_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SplineFollower>();
    node->startDrawing();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
