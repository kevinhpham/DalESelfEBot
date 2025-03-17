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

#include "spline_follower.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SplineFollower>();
    node->on_activate();  // Starts the state machine

    rclcpp::spin(node);   // Keep spinning while the state machine runs

    rclcpp::shutdown();
    return 0;
}
