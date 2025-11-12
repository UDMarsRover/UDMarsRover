#include "mainwindow.h" // Your main window UI
#include "touch_ui_node.h" // ROS publisher wrapper
#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char *argv[])
{
    // 1. Initialize ROS
    rclcpp::init(argc, argv);
    
    // 2. Create UiNode and pass to MainWindow so we can publish messages.
    auto ui_node = std::make_shared<UiNode>();

    // 3. Create the Qt Application
    QApplication a(argc, argv);

    // 4. Create and show your main window, passing the node to it
    MainWindow w(ui_node); // UI wired to ROS publisher
    w.show();

    // 5. Spin the ROS node in a separate thread
    // We use std::thread to run the blocking rclcpp::spin()
    std::thread ros_thread([&]() {
        rclcpp::spin(ui_node->get_node());
    });
    
    // 6. Run the Qt event loop (this is blocking)
    int result = a.exec();

    // 7. After the window is closed, shut down ROS
    rclcpp::shutdown();
    ros_thread.join(); // Wait for the spinner thread to finish
    
    return result;
}