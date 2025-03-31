#include "spline_follower.hpp"
#include <fstream>
#include <thread>

using namespace std::chrono_literals;
using json = nlohmann::json;

// Constructor - init member variables: Node name to: spline_follower, state starts in init and initial spline index is 0
SplineFollower::SplineFollower() : Node("spline_follower"), state_(State::INIT), current_spline_index_(0), flag_received_(false) {
    RCLCPP_INFO(this->get_logger(), "SplineFollower node created.");

    // Initalise Publishers and Subscribers for communication with other subsystems
    toolpath_sub_ = this->create_subscription<std_msgs::msg::Empty>("/toolpath", 10, std::bind(&SplineFollower::toolpath_sub_callback, this, std::placeholders::_1));
    error_pub_ = this->create_publisher<std_msgs::msg::String>("/control_error", 10);
}

// Add in a plane at z = 0 so that moveit avoids colliding with the robot base
void SplineFollower::addGroundPlane() {
    // Declare a planning scene and collision object
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::msg::CollisionObject ground;
    ground.header.frame_id = "world"; // Set frame id to world frame
    ground.id = "ground_plane"; // Name our collision object ground_plane

    // Create a box geometry to be our ground
    shape_msgs::msg::SolidPrimitive ground_shape;
    ground_shape.type = shape_msgs::msg::SolidPrimitive::BOX; // Set the shape to box
    ground_shape.dimensions = {10.0, 10.0, 0.01}; // Define the dimensions - z thickness is 100 mm

    // Create a pose for the ground box to be placed at
    geometry_msgs::msg::Pose ground_pose;
    ground_pose.position.z = -0.01; // Set the z position to 100 mm below 0 (ensures it does not collide with base link)
    
    // Apply box and pose to the collision object
    ground.primitives.push_back(ground_shape);
    ground.primitive_poses.push_back(ground_pose);
    ground.operation = ground.ADD;

    // Apply collision object to our planning scene
    planning_scene_interface.applyCollisionObject(ground);
}

// Load the testing splines from a json file
bool SplineFollower::loadSplines() {
    std::ifstream file("/home/jarred/git/DalESelfEBot/ur3_control/config/circle_waypoints.json");
    if (!file) {
        RCLCPP_ERROR(this->get_logger(), "Could not open spline JSON file.");
        return false;
    }

    file >> spline_data_;
    return !spline_data_["splines"].empty();
}

// Set a safe start pose directly above the centre of the canvas
void SplineFollower::setSafeStartPose() {
    // Define the YAML file path from localisation package
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

    // Compute the center position of the localisation rectangle
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

// Compute a set of waypoints in straight line between two poses
const std::vector<geometry_msgs::msg::Pose> SplineFollower::computeLinearInterpolationPath(
    const geometry_msgs::msg::Pose& start_pose, 
    const geometry_msgs::msg::Pose& end_pose, 
    int num_waypoints) 
{
    // Declare the waypoints vector
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(num_waypoints); // Reserve the desire amount of waypoints

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

        waypoints.push_back(interpolated_pose);
    }

    return waypoints;
}

// Calculate the average canvas height based on localisation - to use as the z position for spline drawing
double SplineFollower::calculateAverageCanvasHeight() {
    // Define the YAML file path from localisation package
    std::string yaml_path = "/home/jarred/git/DalESelfEBot/ur3_localisation/config/params.yaml";
    
    // Open the YAML file
    YAML::Node config = YAML::LoadFile(yaml_path);
    if (!config["corner_positions"]) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load corner_positions from params.yaml!");
        return 0.05;  // Default canvas height if loading fails
    }

    // Calculate average canvas height
    double total_z = 0.0;
    int count = 0;
    for (const auto& corner : config["corner_positions"]) { // Loop through positions
        total_z += corner["z"].as<double>(); // Aggregate the z values
        count++;
    }

    return (count > 0) ? (total_z / count) : 0.05;  // Return the average of the aggregate - Default to 50mm if something goes wrong
}

void SplineFollower::toolpath_sub_callback(const std_msgs::msg::Empty::SharedPtr msg){

    std::unique_lock<std::mutex> lock(mtx_);
    // RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    // Set the flag and notify main thread
    flag_received_ = true;
    cv_.notify_one();
    process_toolath_to_json();

}

void SplineFollower::process_toolath_to_json(){
    // Add logic here to process Amriths toolpath data into the json format used in ur3_control
}

// Function to publish error message to GUI
void SplineFollower::sendError(bool drawing_incomplete) {
    std_msgs::msg::String msg; // Create a string message

    if(drawing_incomplete){ // If the drawing was not completed advise to retake the selfie
        msg.data = "Control Failed! The toolpaths could not be completed. Please retake the selfie.";
    }
    else{ // If the drawing was completed advise to return robot home manually
        msg.data = "Control Failed! The toolpaths were completed. Please return robot to the upright posistion manually.";
    }
    error_pub_->publish(msg); // Publish message
}

// Function to generate a border spline and place it at position 0 to be drawn first
bool SplineFollower::generateBorderSpline(double offset) { // Offset in metres
    // Load the YAML file with corner positions
    YAML::Node config = YAML::LoadFile("/home/jarred/git/DalESelfEBot/ur3_localisation/config/params.yaml");
    if (!config["corner_positions"] || config["corner_positions"].size() != 4) {
        RCLCPP_ERROR(this->get_logger(), "Invalid or missing corner_positions in YAML.");
        return false;
    }

    // Extract corners
    std::vector<std::array<double, 3>> corners;
    for (const auto& corner : config["corner_positions"]) {
        double x = corner["x"].as<double>();
        double y = corner["y"].as<double>();
        double z = corner["z"].as<double>();
        corners.push_back({x, y, z});
    }

    if (corners.size() != 4) {
        RCLCPP_ERROR(this->get_logger(), "Exactly four corners required.");
        return false;
    }

    // Detect winding order
    double signed_area = 0.0;
    for (int i = 0; i < 4; ++i) {
        const auto& p1 = corners[i];
        const auto& p2 = corners[(i + 1) % 4];
        signed_area += (p1[0] * p2[1] - p2[0] * p1[1]);
    }
    bool is_clockwise = signed_area < 0;
    double sign = is_clockwise ? -1.0 : 1.0;

    // Compute offset corners using average normals
    std::vector<std::array<double, 3>> offset_corners;
    for (size_t i = 0; i < 4; ++i) {
        const auto& prev = corners[(i + 3) % 4];
        const auto& curr = corners[i];
        const auto& next = corners[(i + 1) % 4];

        // Vectors for adjacent edges
        double dx1 = curr[0] - prev[0];
        double dy1 = curr[1] - prev[1];
        double len1 = std::hypot(dx1, dy1);
        dx1 /= len1;
        dy1 /= len1;

        double dx2 = next[0] - curr[0];
        double dy2 = next[1] - curr[1];
        double len2 = std::hypot(dx2, dy2);
        dx2 /= len2;
        dy2 /= len2;

        // Normals for each edge
        double nx1 = -dy1;
        double ny1 = dx1;
        double nx2 = -dy2;
        double ny2 = dx2;

        // Average normal
        double nx = nx1 + nx2;
        double ny = ny1 + ny2;
        double norm_len = std::hypot(nx, ny);
        if (norm_len > 1e-6) {
            nx /= norm_len;
            ny /= norm_len;
        }

        double x = curr[0] + sign * nx * offset;
        double y = curr[1] + sign * ny * offset;
        double z = curr[2];
        offset_corners.push_back({x, y, z});
    }

    // Generate interpolated points between offset corners
    std::vector<std::vector<double>> border_waypoints;
    for (size_t i = 0; i < 4; ++i) {
        const auto& p1 = offset_corners[i];
        const auto& p2 = offset_corners[(i + 1) % 4];

        double dx = p2[0] - p1[0];
        double dy = p2[1] - p1[1];
        double length = std::hypot(dx, dy);
        size_t steps = std::max<size_t>(2, static_cast<size_t>(length / 0.01));

        for (size_t j = 0; j <= steps; ++j) {
            double t = static_cast<double>(j) / steps;
            double x = p1[0] + t * dx;
            double y = p1[1] + t * dy;
            double z = (p1[2] + p2[2]) / 2.0;
            border_waypoints.push_back({x, y, z});
        }
    }

    // Close the loop
    border_waypoints.push_back(border_waypoints.front());

    // Create border spline JSON
    nlohmann::json border_spline;
    border_spline["id"] = 0;
    border_spline["waypoints"] = border_waypoints;

    if (!spline_data_.contains("splines")) {
        spline_data_["splines"] = nlohmann::json::array();
    }

    if (!spline_data_["splines"].empty() && spline_data_["splines"][0]["id"] == 0) {
        spline_data_["splines"].erase(spline_data_["splines"].begin());
    }

    spline_data_["splines"].insert(spline_data_["splines"].begin(), border_spline);
    RCLCPP_INFO(this->get_logger(), "Border spline generated and inserted at index 0.");
    return true;
}

void SplineFollower::exportSplineToCSV(const std::string& filename) {

    // Load the YAML file with corner positions
    YAML::Node config = YAML::LoadFile("/home/jarred/git/DalESelfEBot/ur3_localisation/config/params.yaml");
    if (!config["corner_positions"] || config["corner_positions"].size() != 4) {
        RCLCPP_ERROR(this->get_logger(), "Invalid or missing corner_positions in YAML.");
        return;
    }

    // Extract corners and apply offset
    std::vector<std::array<double, 3>> corners;
    for (const auto& corner : config["corner_positions"]) {
        double x = corner["x"].as<double>();
        double y = corner["y"].as<double>();
        double z = corner["z"].as<double>();
        corners.push_back({x, y, z});
    }

    std::ofstream file(filename);
    file << "type,id,x,y,z\n";

    // Export corner positions
    for (const auto& corner : corners) {
        file << "corner,NA," << corner[0] << "," << corner[1] << "," << corner[2] << "\n";
    }

    // Export all splines
    if (spline_data_.contains("splines") && !spline_data_["splines"].empty()) {
        for (const auto& spline : spline_data_["splines"]) {
            int id = spline["id"].get<int>();
            for (const auto& wp : spline["waypoints"]) {
                file << "spline," << id << "," << wp[0] << "," << wp[1] << "," << wp[2] << "\n";
            }
        }
    }

    file.close();
    RCLCPP_INFO(this->get_logger(), "Exported all splines and localisation to %s", filename.c_str());
}

bool SplineFollower::generateSignageSpline() {
    // Choose a starting position in the lower left of the canvas
    if (!spline_data_.contains("splines") || spline_data_["splines"].empty()) {
        RCLCPP_ERROR(this->get_logger(), "No splines loaded. Please run loadSplines or generateBorderSpline first.");
        return false;
    }

    // Use the first corner to base the logo placement
    const auto& base_point = spline_data_["splines"][0]["waypoints"][0];
    double base_x = base_point[0];
    double base_y = base_point[1];
    double base_z = base_point[2];

    // Scale for the size of the letters
    double scale = 0.03; // 3 cm letters

    // Offsets for a simple block "DALE" logo
    std::vector<std::vector<std::vector<double>>> letters = {
        // D
        {
            {0,0,0}, {0,1,0}, {0.5,1,0}, {0.6,0.9,0}, {0.6,0.1,0}, {0.5,0,0}, {0,0,0}
        },
        // A
        {
            {1.0,0,0}, {1.3,1.0,0}, {1.6,0,0}, {1.45,0.5,0}, {1.15,0.5,0}
        },
        // L
        {
            {2.0,1.0,0}, {2.0,0,0}, {2.5,0,0}
        },
        // E
        {
            {3.0,1.0,0}, {3.0,0,0}, {3.6,0,0}, {3.0,0.5,0}, {3.4,0.5,0}, {3.0,1.0,0}, {3.6,1.0,0}
        }
    };

    // Shift and scale each letter
    int next_id = spline_data_["splines"].size();
    for (const auto& letter : letters) {
        std::vector<std::vector<double>> wp;
        for (const auto& p : letter) {
            wp.push_back({
                base_x + p[0] * scale,
                base_y + p[1] * scale,
                base_z
            });
        }
        nlohmann::json spline;
        spline["id"] = next_id++;
        spline["waypoints"] = wp;
        spline_data_["splines"].push_back(spline);
    }

    RCLCPP_INFO(this->get_logger(), "Logo spline (DALE) added to spline data.");
    return true;
}
