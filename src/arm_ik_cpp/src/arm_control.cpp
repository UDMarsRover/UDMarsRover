#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

const float MAX_STEPS_PER_UPDATE = 10.0f;

class ArmControl : public rclcpp::Node
{
public:

    ArmControl()
        : Node("arm_control")
    {
        //--------------------------------------------------
        // Publisher
        //--------------------------------------------------

        arm_pub_ =
            create_publisher<
                std_msgs::msg::Float32MultiArray>(
                    "/arm_command",
                    10);

        //--------------------------------------------------
        // Subscriber
        //--------------------------------------------------

        joy_sub_ =
            create_subscription<
                sensor_msgs::msg::Joy>(
                    "/arm/joy", //need to run the following command to remap the joy topic: ros2 run joy joy_node --ros-args --remap /joy:=/arm/joy
                    10,
                    std::bind(
                        &ArmControl::joyCallback,
                        this,
                        std::placeholders::_1));
    }

private:

    float target_position_[6] =
{
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f
};

    void joyCallback(
        const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        std_msgs::msg::Float32MultiArray command;

        command.data.resize(6);

        //------------------------------------------
        // Map controls here
        //------------------------------------------

        target_position_[0] += msg->axes[0] * MAX_STEPS_PER_UPDATE;   // Turret
        target_position_[1] += msg->axes[1] * MAX_STEPS_PER_UPDATE * 5;   // Shoulder
        target_position_[2] += msg->axes[3] * MAX_STEPS_PER_UPDATE * 5;   // Elbow
        target_position_[3] += msg->axes[2] * MAX_STEPS_PER_UPDATE * 8;   // Gripper
        if(msg->buttons[8] == 1)
        {
            target_position_[4] = 50.0f;   // Wrist
        }
        else if(msg->buttons[7] == 1)
        {
            target_position_[4] = -50.0f;   // Wrist
        }
        else
        {
            target_position_[4] = 0.0f;   // Wrist
        }

        if(msg->buttons[6] == 1)
        {
            target_position_[5] += 50.0f;   // linear act
        }
        else if(msg->buttons[5] == 1)
        {
            target_position_[5] += -50.0f;   // linear act
        }
        

        publishCommand();
    }

    void publishCommand()
    {
        std_msgs::msg::Float32MultiArray command;

        command.data.resize(6);
        
        for (int i = 0; i < 6; i++)
        {
            command.data[i] = target_position_[i];
        }

        arm_pub_->publish(command);
    }
    //--------------------------------------------------
    // ROS Objects
    //--------------------------------------------------

    rclcpp::Subscription<
        sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    rclcpp::Publisher<
        std_msgs::msg::Float32MultiArray>::SharedPtr arm_pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<ArmControl>());

    rclcpp::shutdown();

    return 0;
}