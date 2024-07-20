#include <stdio.h>
#include <memory>
#include "utils/Logger.hpp"
#include "config/Config.hpp"

#include "io/ros/RosPublisher.hpp"


void start_directory_publisher(vstk::VstkConfig conf) {
    INFOLOG("Creating directory source publisher for stereo images");
    std::shared_ptr<vstk::RosImageDirPublisher> directory_publisher = std::make_shared<vstk::RosImageDirPublisher>(conf);
    directory_publisher->initialize();

    rclcpp::executors::MultiThreadedExecutor mt_executor;
    mt_executor.add_node(directory_publisher);
    mt_executor.spin();
}

int main(int argc, char **argv) {
    INFOLOG("Starting stereo publisher ...");
    rclcpp::init(0, nullptr);
    vstk::VstkConfig conf = vstk::build_config_from_args(argc, argv);
    start_directory_publisher(conf);
    rclcpp::shutdown();
}