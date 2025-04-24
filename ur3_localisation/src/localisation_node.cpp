#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <fstream>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sys/types.h>
#include <pwd.h>

class UR3Localisation : public rclcpp::Node {
public:
    UR3Localisation() : Node("ur3_localisation"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) { // Initialise memeber variables
        freedrive_client_ = this->create_client<std_srvs::srv::Trigger>("/io_and_status_controller/set_freedrive"); // Create client to set robot into freedrive mode
        save_subscriber_ = this->create_subscription<std_msgs::msg::Empty>( // Create subscriber for saving each position
            "/save_position", 10, std::bind(&UR3Localisation::save_position_callback, this, std::placeholders::_1));
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10); // Create marker publisher

        struct passwd *pw = getpwuid(getuid());
        std::string home_path = pw->pw_dir;
        save_path_ = home_path + "/ros2_ws/src/ur3_localisation/config/params.yaml"; // Save Yaml path for storing locations

        RCLCPP_INFO(this->get_logger(), "UR3 Localisation Node Ready.");
        RCLCPP_INFO(this->get_logger(), "Saving positions to: %s", save_path_.c_str());
    }

private:
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr freedrive_client_; // Client for setting robot into freedrive mode
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr save_subscriber_; // Subscriber for saving locations
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_; // Publisher for markers 
    std::vector<Eigen::Vector3d> positions_; // Vector for storing each position
    std::vector<Eigen::Quaterniond> orientations_; // Vector for storing each orientation
    std::string save_path_; // String to save yaml path
    tf2_ros::Buffer tf_buffer_; // Buffer for the tf listener
    tf2_ros::TransformListener tf_listener_; // Transform listener to fetch the pose

    // Callback function to save each position as we perform localisation
    void save_position_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/) {
        try {
            // Get transform from base_link to tool0
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_.lookupTransform("base_link", "tool0", tf2::TimePointZero);
    
            // Convert to Eigen for manipulation
            Eigen::Vector3d tool_position(transform_stamped.transform.translation.x,
                                          transform_stamped.transform.translation.y,
                                          transform_stamped.transform.translation.z);
    
            Eigen::Quaterniond tool_orientation(transform_stamped.transform.rotation.w,
                                                transform_stamped.transform.rotation.x,
                                                transform_stamped.transform.rotation.y,
                                                transform_stamped.transform.rotation.z);
    
            // Create 108 mm Z offset in the TOOL frame
            Eigen::Vector3d z_offset_tool(0.0, 0.0, 0.108);  // 108 mm = 0.108 m
    
            // Rotate the offset into the base frame
            Eigen::Vector3d offset_base = tool_orientation * z_offset_tool;
    
            // Apply offset in base frame
            Eigen::Vector3d position_with_offset = tool_position + offset_base;
    
            // Save and publish
            if (positions_.size() < 4) {
                positions_.push_back(position_with_offset);
                orientations_.push_back(tool_orientation);
                publish_marker(position_with_offset, positions_.size(), 0.0, 0.0, 1.0, 0.05);
    
                RCLCPP_INFO(this->get_logger(), "Position %ld saved (with offset): x=%.3f, y=%.3f, z=%.3f",
                            positions_.size(), position_with_offset.x(), position_with_offset.y(), position_with_offset.z());
    
                if (positions_.size() == 4) {
                    save_positions();
                    publish_plane_marker();
                    disable_freedrive();
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "All 4 positions have already been saved.");
            }
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }    

    // Publishes a sphere marker at each location to confirm the point is consistent with the end effector position in simulation
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

    // Publishes a plane visualising the canvas plane
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

    // Saves all positions to the yaml once localisation is complete
    void save_positions() {
        YAML::Node yaml_data;
        for (size_t i = 0; i < positions_.size(); ++i) {
            yaml_data["corner_positions"][i]["x"] = positions_[i].x();
            yaml_data["corner_positions"][i]["y"] = positions_[i].y();
            yaml_data["corner_positions"][i]["z"] = positions_[i].z();
            yaml_data["corner_positions"][i]["qx"] = orientations_[i].x();
            yaml_data["corner_positions"][i]["qy"] = orientations_[i].y();
            yaml_data["corner_positions"][i]["qz"] = orientations_[i].z();
            yaml_data["corner_positions"][i]["qw"] = orientations_[i].w();
        }
        std::ofstream fout(save_path_);
        fout << yaml_data;
        fout.close();
        RCLCPP_INFO(this->get_logger(), "Positions saved successfully to %s", save_path_.c_str());
    }

    // Disables the freedrive mode
    void disable_freedrive() {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        freedrive_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Freedrive Mode DISABLED.");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv); // Initialise ros
    auto node = std::make_shared<UR3Localisation>(); // Crete a shared pointer ro UR3Localisation - process continues in callback function
    rclcpp::spin(node);
    rclcpp::shutdown(); // Shutdown when complete 
    return 0;
}
