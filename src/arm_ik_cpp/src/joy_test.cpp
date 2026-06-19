#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoyTest : public rclcpp::Node
{
public:
    JoyTest() : Node("joy_test")
    {
        sub_ = create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            10,
            std::bind(&JoyTest::callback, this, std::placeholders::_1)
        );
    }

private:
    void callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        RCLCPP_INFO(
            get_logger(),
            "Axes: %.2f %.2f %.2f %.2f",
            msg->axes[0],
            msg->axes[1],
            msg->axes[2],
            msg->axes[3]
        );
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyTest>());
    rclcpp::shutdown();
}