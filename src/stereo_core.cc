#include <stdlib.h>
#include <stdio.h>

#include "utils/Logger.hpp"
#include "config/Config.hpp"
#include "io/ros/RosPipeline.hpp"

typedef std::shared_ptr<vstk::VstkCore> VstkCorePtr;
#define MAKE_CORE(CNF) std::make_shared<vstk::VstkCore>(CNF)

int main(int argc, char **argv) {
    INFOLOG("Starting vstk core on ROS"); 
    rclcpp::init(0, nullptr);
    
    rclcpp::executors::MultiThreadedExecutor mt_executor;
    vstk::VstkConfig conf = vstk::build_config_from_args(argc, argv);
    VstkCorePtr core_ptr = MAKE_CORE(conf);
    vstk::describe_config(conf);

    mt_executor.add_node(core_ptr);
    mt_executor.spin();
    rclcpp::shutdown();

    return 0;
}
