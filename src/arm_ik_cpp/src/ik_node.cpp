#include <fstream>
#include <sstream>
#include <memory>
#include <chrono>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <trac_ik/trac_ik.hpp>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

using namespace std::chrono_literals;

class IKNode : public rclcpp::Node
{
public:
    IKNode()
    : Node("ik_node")
    {
        //------------------------------------------
        // Load robot description
        //------------------------------------------
        loadURDF();

        //------------------------------------------
        // Create ROS interfaces
        //------------------------------------------
        createPublisher();
        createSubscriber();

        //------------------------------------------
        // Initial target position
        //------------------------------------------
        target_x_ = 0.50;
        target_y_ = 0.00;
        target_z_ = 0.20;

        //------------------------------------------
        // Delay IK initialization until after the
        // node is fully constructed.
        //------------------------------------------
        startup_timer_ = create_wall_timer(
            1ms,
            std::bind(&IKNode::initializeIK, this));

        RCLCPP_INFO(get_logger(), "IK Node started.");
    }

private:

    //-------------------------------------------------
    // Load URDF into robot_description parameter
    //-------------------------------------------------

    void loadURDF()
    {
        std::string package_path =
            ament_index_cpp::get_package_share_directory("arm");

        std::ifstream urdf_file(
            package_path + "/urdf/robot.urdf");

        if (!urdf_file.is_open())
        {
            RCLCPP_FATAL(
                get_logger(),
                "Unable to load robot.urdf");

            return;
        }

        std::stringstream buffer;
        buffer << urdf_file.rdbuf();

        declare_parameter(
            "robot_description",
            buffer.str());

        RCLCPP_INFO(
            get_logger(),
            "Loaded URDF.");
    }

    //-------------------------------------------------
    // One-shot initialization
    //-------------------------------------------------

    void initializeIK()
    {
        //------------------------------------------
        // Make timer fire only once
        //------------------------------------------

        startup_timer_->cancel();

        //------------------------------------------
        // Construct TRAC-IK
        //------------------------------------------

        ik_solver_ =
            std::make_unique<TRAC_IK::TRAC_IK>(
                shared_from_this(),
                "base_link",
                "wrist_link",
                "robot_description",
                0.005,
                1e-5);

        //------------------------------------------
        // Build KDL chain
        //------------------------------------------

        if (!ik_solver_->getKDLChain(chain_))
        {
            RCLCPP_FATAL(
                get_logger(),
                "Failed to build KDL chain.");

            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "TRAC-IK initialized.");

        RCLCPP_INFO(
            get_logger(),
            "Chain contains %u joints.",
            chain_.getNrOfJoints());
    }

    //-------------------------------------------------
    // Publisher
    //-------------------------------------------------

    void createPublisher()
    {
        joint_pub_ =
            create_publisher<
                sensor_msgs::msg::JointState>(
                    "/joint_targets",
                    10);
    }

    //-------------------------------------------------
    // Subscriber
    //-------------------------------------------------

    void createSubscriber()
    {
        joy_sub_ =
            create_subscription<
                sensor_msgs::msg::Joy>(
                    "/joy",
                    10,
                    std::bind(
                        &IKNode::joyCallback,
                        this,
                        std::placeholders::_1));
    }

    //-------------------------------------------------
    // Joystick callback
    //-------------------------------------------------

    void joyCallback(
        const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (!ik_solver_)
        {
            return;
        }

        if (msg->axes.size() < 4)
        {
            return;
        }

        //------------------------------------------
        // Adjust desired end-effector position
        //------------------------------------------

        target_x_ += msg->axes[1] * 0.01;
        target_y_ += msg->axes[0] * 0.01;
        target_z_ += msg->axes[3] * 0.01;

        solveIK();
    }

    //-------------------------------------------------
    // Solve inverse kinematics
    //-------------------------------------------------

    void solveIK()
    {
        KDL::Frame target(
            KDL::Vector(
                target_x_,
                target_y_,
                target_z_));

        KDL::JntArray seed(chain_.getNrOfJoints());
        KDL::JntArray result(chain_.getNrOfJoints());

        int rc =
            ik_solver_->CartToJnt(
                seed,
                target,
                result);

        if (rc < 0)
        {
            RCLCPP_WARN(
                get_logger(),
                "No IK solution found.");

            return;
        }

        sensor_msgs::msg::JointState joints;

        joints.header.stamp = now();

        joints.name =
        {
            "turret_joint",
            "shoulder_joint",
            "elbow_joint",
            "wrist_joint"
        };

        for (unsigned int i = 0;
             i < result.rows();
             i++)
        {
            joints.position.push_back(result(i));
        }

        joint_pub_->publish(joints);

        RCLCPP_INFO(
            get_logger(),
            "Target: %.3f %.3f %.3f",
            target_x_,
            target_y_,
            target_z_);
    }

    //-------------------------------------------------
    // ROS Interfaces
    //-------------------------------------------------

    rclcpp::Subscription<
        sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    rclcpp::Publisher<
        sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    rclcpp::TimerBase::SharedPtr startup_timer_;

    //-------------------------------------------------
    // TRAC-IK
    //-------------------------------------------------

    std::unique_ptr<TRAC_IK::TRAC_IK> ik_solver_;

    KDL::Chain chain_;

    //-------------------------------------------------
    // Target position
    //-------------------------------------------------

    double target_x_;
    double target_y_;
    double target_z_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<IKNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}