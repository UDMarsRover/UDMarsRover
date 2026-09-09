#include <fstream>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <trac_ik/trac_ik.hpp>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

class IKNode : public rclcpp::Node
{
public:

    IKNode() : Node("ik_node")
    {
        loadURDF();

        joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            "/joint_targets", 10);

        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            10,
            std::bind(&IKNode::joyCallback, this, std::placeholders::_1));
    }

    void initializeIK()
    {
        ik_solver_ = std::make_unique<TRAC_IK::TRAC_IK>(
            shared_from_this(),
            "base_link",
            "wrist_link",
            "robot_description",
            0.005,
            1e-5
        );

        if (!ik_solver_->getKDLChain(chain_))
        {
            RCLCPP_FATAL(get_logger(), "Failed to build KDL chain.");
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "Chain contains %u joints",
            chain_.getNrOfJoints());
    }


private:

    void joyCallback(
        const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        //------------------------------------------
        // Increment target position
        //------------------------------------------

        target_x_ += msg->axes[1] * 0.01;
        target_y_ += msg->axes[0] * 0.01;
        target_z_ += msg->axes[3] * 0.01;

        solveIK();
    }

    void solveIK()
    {
        KDL::Frame target(
            KDL::Vector(
                target_x_,
                target_y_,
                target_z_
            )
        );

        KDL::JntArray seed(
            chain_.getNrOfJoints());

        KDL::JntArray result(
            chain_.getNrOfJoints());

        int rc =
            ik_solver_->CartToJnt(
                seed,
                target,
                result);

        if(rc < 0)
        {
            RCLCPP_WARN(
                get_logger(),
                "IK Failed");
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

        for(unsigned int i=0;i<result.rows();i++)
        {
            joints.position.push_back(result(i));
        }

        joint_pub_->publish(joints);

        RCLCPP_INFO(
            get_logger(),
            "Target: %.2f %.2f %.2f",
            target_x_,
            target_y_,
            target_z_);
    }

    //------------------------------------------

    rclcpp::Subscription<
        sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    rclcpp::Publisher<
        sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    //------------------------------------------

    std::unique_ptr<TRAC_IK::TRAC_IK> ik_solver_;

    KDL::Chain chain_;

    //------------------------------------------

    double target_x_;
    double target_y_;
    double target_z_;
};

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);

    rclcpp::spin(
        std::make_shared<IKNode>());

    rclcpp::shutdown();
}