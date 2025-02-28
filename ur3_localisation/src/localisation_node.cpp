#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sys/types.h>
#include <pwd.h>

class UR3Localisation : public rclcpp::Node {
public:
    UR3Localisation() : Node("ur3_localisation") {
        // Freedrive service client
        freedrive_client_ = this->create_client<std_srvs::srv::Trigger>("/io_and_status_controller/set_freedrive");

        // Joint state subscriber
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&UR3Localisation::joint_state_callback, this, std::placeholders::_1));

        // Save trigger subscriber
        save_subscriber_ = this->create_subscription<std_msgs::msg::Empty>(
            "/save_position", 10, std::bind(&UR3Localisation::save_position_callback, this, std::placeholders::_1));

        // Get package path dynamically
        struct passwd *pw = getpwuid(getuid());
        std::string home_path = pw->pw_dir;
        save_path_ = home_path + "/ros2_ws/src/ur3_localisation/config/params.yaml";

        RCLCPP_INFO(this->get_logger(), "UR3 Localisation Node Ready.");
        RCLCPP_INFO(this->get_logger(), "Saving positions to: %s", save_path_.c_str());
    }

private:
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr freedrive_client_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr save_subscriber_;
    
    Eigen::Vector3d end_effector_position_;
    Eigen::Quaterniond end_effector_orientation_;
    
    std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> positions_;  // Store (position, orientation)
    std::string save_path_;

    // **Corrected DH Parameters**: {theta (offset), a, d, alpha}
    const std::vector<std::vector<double>> DH_PARAMS = {
        {0,         0.0,        0.1519,   M_PI / 2},   // Joint 1
        {0,    -0.24365,       0.0,          0},       // Joint 2
        {0,    -0.21325,       0.0,          0},       // Joint 3
        {0,         0.0,    0.11235,   M_PI / 2},      // Joint 4
        {0,         0.0,    0.08535,  -M_PI / 2},      // Joint 5
        {0,         0.0,    0.0819,         0}        // Joint 6 (End-effector)
    };

    // Compute transformation matrix using **Denavit-Hartenberg (DH)**
    Eigen::Matrix4d compute_dh_matrix(double theta, double a, double d, double alpha) {
        Eigen::Matrix4d T;
        T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
             sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
             0, sin(alpha), cos(alpha), d,
             0, 0, 0, 1;
        return T;
    }

    // **Forward Kinematics Calculation**
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->position.size() < 6) {
            RCLCPP_WARN(this->get_logger(), "Joint state message has insufficient values.");
            return;
        }

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        for (size_t i = 0; i < 6; ++i) {
            double theta = msg->position[i] + DH_PARAMS[i][0];  // Joint angle + DH offset
            double a = DH_PARAMS[i][1];
            double d = DH_PARAMS[i][2];
            double alpha = DH_PARAMS[i][3];
            T *= compute_dh_matrix(theta, a, d, alpha);
        }

        // Extract translation (XYZ)
        end_effector_position_ = T.block<3, 1>(0, 3);

        // Extract rotation (convert to quaternion)
        Eigen::Matrix3d rotation_matrix = T.block<3, 3>(0, 0);
        end_effector_orientation_ = Eigen::Quaterniond(rotation_matrix);
    }

    // Save end-effector position & orientation when triggered
    void save_position_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/) {
        if (positions_.size() < 4) {
            positions_.emplace_back(end_effector_position_, end_effector_orientation_);
            
            RCLCPP_INFO(this->get_logger(), 
                "Position %ld saved: x=%.3f, y=%.3f, z=%.3f | qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f", 
                positions_.size(), 
                end_effector_position_.x(), end_effector_position_.y(), end_effector_position_.z(),
                end_effector_orientation_.x(), end_effector_orientation_.y(), end_effector_orientation_.z(), end_effector_orientation_.w()
            );

            if (positions_.size() == 4) {
                save_positions();
                disable_freedrive();
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "All 4 positions have already been saved.");
        }
    }

    // Save positions to YAML file
    void save_positions() {
        YAML::Node yaml_data;
        for (size_t i = 0; i < positions_.size(); ++i) {
            yaml_data["corner_positions"][i]["x"] = positions_[i].first.x();
            yaml_data["corner_positions"][i]["y"] = positions_[i].first.y();
            yaml_data["corner_positions"][i]["z"] = positions_[i].first.z();
            yaml_data["corner_positions"][i]["qx"] = positions_[i].second.x();
            yaml_data["corner_positions"][i]["qy"] = positions_[i].second.y();
            yaml_data["corner_positions"][i]["qz"] = positions_[i].second.z();
            yaml_data["corner_positions"][i]["qw"] = positions_[i].second.w();
        }

        std::ofstream fout(save_path_);
        fout << yaml_data;
        fout.close();

        RCLCPP_INFO(this->get_logger(), "Positions & orientations saved successfully to %s", save_path_.c_str());
    }

    // Disable freedrive mode
    void disable_freedrive() {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        freedrive_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Freedrive Mode DISABLED.");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UR3Localisation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
