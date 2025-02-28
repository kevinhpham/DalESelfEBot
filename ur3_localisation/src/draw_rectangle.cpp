#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

class UR3DrawRectangle : public rclcpp::Node {
public:
    UR3DrawRectangle() : Node("ur3_draw_rectangle") {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/cartesian_goal", 10);

        // Load positions from YAML file
        YAML::Node config = YAML::LoadFile("config/params.yaml");
        for (const auto & pos : config["corner_positions"]) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = pos["x"].as<double>();
            pose.position.y = pos["y"].as<double>();
            pose.position.z = pos["z"].as<double>();
            positions_.push_back(pose);
        }

        RCLCPP_INFO(this->get_logger(), "Starting rectangle drawing...");
        draw_rectangle();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    std::vector<geometry_msgs::msg::Pose> positions_;

    void draw_rectangle() {
        for (const auto & pose : positions_) {
            geometry_msgs::msg::PoseStamped msg;
            msg.pose = pose;
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Moving to: (%.2f, %.2f, %.2f)", pose.position.x, pose.position.y, pose.position.z);
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        RCLCPP_INFO(this->get_logger(), "Rectangle drawn successfully.");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UR3DrawRectangle>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
