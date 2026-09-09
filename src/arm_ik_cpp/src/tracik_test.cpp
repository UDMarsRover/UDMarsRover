#include <fstream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

#include <trac_ik/trac_ik.hpp>
#include <kdl/chain.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        rclcpp::Node::make_shared("tracik_test");

    std::ifstream urdf_file(
        "/home/tom/Workspaces/UDMarsRover/src/arm/urdf/robot.urdf"
    );

    if (!urdf_file.is_open())
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Could not open URDF file"
        );

        return 1;
    }

    std::stringstream buffer;
    buffer << urdf_file.rdbuf();

    std::string urdf_string = buffer.str();

    node->declare_parameter(
        "robot_description",
        urdf_string
    );

    auto ik_solver =
        TRAC_IK::TRAC_IK(
            node,
            "base_link",
            "wrist_link",
            "robot_description",
            0.005,
            1e-5
        );

    KDL::Chain chain;

    if (!ik_solver.getKDLChain(chain))
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Failed to build KDL chain"
        );

        return 1;
    }

    RCLCPP_INFO(
        node->get_logger(),
        "TRAC-IK loaded successfully"
    );

    RCLCPP_INFO(
        node->get_logger(),
        "Joint count: %d",
        chain.getNrOfJoints()
    );

    rclcpp::shutdown();

    return 0;
}