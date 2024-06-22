#ifndef __VSTK_ROS_LOADER_H
#define __VSTK_ROS_LOADER_H

#include <stdio.h>
#include <vector>
#include <string>

#include "io/DiskIO.hpp"
#include "io/DataLoader.hpp"

namespace vstk {
    class RosLoader : public DataLoader {
        private:
            std::string ros_topic_name;

        public:
            RosLoader(std::string topic_name);
            RosLoader();
            
            int load_source(std::string topic);
            int iterate_next_image(cv::Mat &image);
            bool is_iteration_complete();
    };
}

#endif