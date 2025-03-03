#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sys/types.h>
#include <pwd.h>

class UR3Localisation : public rclcpp::Node {
public:
    UR3Localisation() : Node("ur3_localisation") {
        freedrive_client_ = this->create_client<std_srvs::srv::Trigger>("/io_and_status_controller/set_freedrive");
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&UR3Localisation::joint_state_callback, this, std::placeholders::_1));
        save_subscriber_ = this->create_subscription<std_msgs::msg::Empty>(
            "/save_position", 10, std::bind(&UR3Localisation::save_position_callback, this, std::placeholders::_1));
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);

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
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    Eigen::Vector3d end_effector_position_;
    std::vector<Eigen::Vector3d> positions_;
    std::string save_path_;

    const std::vector<double> joint_offsets = {
        0.0,  // Joint 1 starts at -1.57, so its offset is +1.57
        M_PI / 2,   // Joint 2 starts at 0.0, so no offset needed
        0.0,  // Joint 3 starts at -1.57, so its offset is +1.57
        0.0,   // Joint 4 starts at 0.0, so no offset needed
        M_PI / 2,   // Joint 5 starts at 0.0, so no offset needed
        M_PI / 2    // Joint 6 starts at 0.0, so no offset needed
    };    

    // **Corrected DH Parameters**: {theta (offset), a, d, alpha}
    const std::vector<std::vector<double>> DH_PARAMS = {
        {0.0,         0.0,        0.1519,   M_PI / 2},   // Joint 1
        {M_PI / 2,    -0.24365,       0.0,          0.0},       // Joint 2
        {0.0,    -0.21325,       0.0,          0.0},       // Joint 3
        {0.0,         0.0,    0.11235,   M_PI / 2},      // Joint 4
        {M_PI / 2,         0.0,    0.08535,  -M_PI / 2},      // Joint 5
        {M_PI / 2,         0.0,    0.0819,         0.0}        // Joint 6 (End-effector)
    };

    Eigen::Matrix4d compute_dh_matrix(double theta, double a, double d, double alpha) {
        Eigen::Matrix4d T;
        T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
             sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
             0, sin(alpha), cos(alpha), d,
             0, 0, 0, 1;
        return T;
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->position.size() < 6) {
            RCLCPP_WARN(this->get_logger(), "Joint state message has insufficient values.");
            return;
        }
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        for (size_t i = 0; i < 6; ++i) {
            double theta = msg->position[i] + joint_offsets[i];  // Apply offset correction
            double a = DH_PARAMS[i][1];
            double d = DH_PARAMS[i][2];
            double alpha = DH_PARAMS[i][3];
            T *= compute_dh_matrix(theta, a, d, alpha);
        }
        end_effector_position_ = T.block<3, 1>(0, 3);
    }        

    void save_position_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/) {
        if (positions_.size() < 4) {
            positions_.push_back(end_effector_position_);
            publish_marker(end_effector_position_, positions_.size(), 0.0, 0.0, 1.0, 0.05);
            RCLCPP_INFO(this->get_logger(), "Position %ld saved: x=%.3f, y=%.3f, z=%.3f",
                        positions_.size(), end_effector_position_.x(), end_effector_position_.y(), end_effector_position_.z());
            if (positions_.size() == 4) {
                save_positions();
                publish_plane_marker();
                disable_freedrive();
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "All 4 positions have already been saved.");
        }
    }

    void publish_marker(const Eigen::Vector3d &position, int id, double r, double g, double b, double scale) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->now();
        marker.ns = "ur3_localisation";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = position.x();
        marker.pose.position.y = position.y();
        marker.pose.position.z = position.z();
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;
        marker.lifetime = rclcpp::Duration::from_seconds(0);
        marker_publisher_->publish(marker);
    }

    void publish_plane_marker() {
        visualization_msgs::msg::Marker plane_marker;
        plane_marker.header.frame_id = "base_link";
        plane_marker.header.stamp = this->now();
        plane_marker.ns = "ur3_localisation";
        plane_marker.id = 100;
        plane_marker.type = visualization_msgs::msg::Marker::CUBE;
        plane_marker.action = visualization_msgs::msg::Marker::ADD;
        Eigen::Vector3d center = (positions_[0] + positions_[1] + positions_[2] + positions_[3]) / 4.0;
        plane_marker.pose.position.x = center.x();
        plane_marker.pose.position.y = center.y();
        plane_marker.pose.position.z = center.z();
        plane_marker.scale.x = fabs(positions_[0].x() - positions_[1].x());
        plane_marker.scale.y = fabs(positions_[1].y() - positions_[2].y());
        plane_marker.scale.z = 0.02; // Small thickness
        plane_marker.color.r = 1.0;
        plane_marker.color.g = 0.0;
        plane_marker.color.b = 0.0;
        plane_marker.color.a = 0.8;
        plane_marker.lifetime = rclcpp::Duration::from_seconds(0);
        marker_publisher_->publish(plane_marker);
    }

    void save_positions() {
        YAML::Node yaml_data;
        for (size_t i = 0; i < positions_.size(); ++i) {
            yaml_data["corner_positions"][i]["x"] = positions_[i].x();
            yaml_data["corner_positions"][i]["y"] = positions_[i].y();
            yaml_data["corner_positions"][i]["z"] = positions_[i].z();
        }
        std::ofstream fout(save_path_);
        fout << yaml_data;
        fout.close();
        RCLCPP_INFO(this->get_logger(), "Positions saved successfully to %s", save_path_.c_str());
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
