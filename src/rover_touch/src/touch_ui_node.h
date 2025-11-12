#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// UiNode: wraps a ROS2 node and provides a simple publish API for the UI.
class UiNode {
public:
  using SharedPtr = std::shared_ptr<UiNode>;

  // Creates underlying rclcpp::Node and a publisher.
  explicit UiNode(const std::string & node_name = "rover_touch_ui",
                  const std::string & topic_name = "ui/button_press");

  // Publish a notification that the button was pressed.
  void publish_button_press(const std::string & label = "button pressed");

  rclcpp::Node::SharedPtr get_node() const { return node_; }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};
