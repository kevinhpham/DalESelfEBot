#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

TEST(ControlNodeTest, SubscribesToToolpathPlanner) {
    auto node = rclcpp::Node::make_shared("test_node");

    std::string target_topic = "/toolpath";
    std::string expected_type = "std_msgs/msg/Empty";

    // Give time for other nodes to start up
    rclcpp::sleep_for(std::chrono::seconds(2));

    auto topic_names_and_types = node->get_topic_names_and_types();

    bool topic_found = false;
    bool type_matches = false;

    for (const auto& entry : topic_names_and_types) {
        if (entry.first == target_topic) {
            topic_found = true;
            for (const auto& type : entry.second) {
                if (type == expected_type) {
                    type_matches = true;
                    break;
                }
            }
            break;
        }
    }

    EXPECT_TRUE(topic_found) << "Topic " << target_topic << " not found";
    EXPECT_TRUE(type_matches) << "Expected type " << expected_type << " not found on topic " << target_topic;
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}

