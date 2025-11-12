#include "touch_ui_node.h"

UiNode::UiNode(const std::string & node_name, const std::string & topic_name)
{
  node_ = std::make_shared<rclcpp::Node>(node_name);
  publisher_ = node_->create_publisher<std_msgs::msg::String>(topic_name, 10);
}

void UiNode::publish_button_press(const std::string & label) {
  std_msgs::msg::String msg;
  msg.data = label;
  publisher_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "Published UI button press: '%s'", msg.data.c_str());
}
