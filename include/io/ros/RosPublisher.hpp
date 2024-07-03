#ifndef __VSTK_ROS_PUBLISHER_H
#define __VSTK_ROS_PUBLISHER_H

#include <stdio.h>
#include <string>
#include <memory>
#include <chrono>

#include <queue>
#include <mutex>
#include <atomic>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.h"

#include "io/DiskIO.hpp"
#include "features/FeatureExtractor.hpp"
#include "utils/Logger.hpp"
#include "utils/CvUtils.hpp"
#include "io/ros/RosDefs.hpp"

namespace vstk {
  
  /*
  * This code needs to read images from a directory and publish them to ROS using probably a message queue 
  * implemented in this class, publish_image should add images to the queue in class and then the private callback method
  * should lock the mutex and check for update in the queue and subsequently publish anything on the queue.
  */

    class RosImageDirPublisher : public rclcpp::Node {
        private:
            uint8_t frame_rate;
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<RosImageMsg>::SharedPtr l_publisher;
            rclcpp::Publisher<RosImageMsg>::SharedPtr r_publisher;
            rclcpp::Publisher<RosCameraInfoMsg>::SharedPtr l_info_publisher;
            rclcpp::Publisher<RosCameraInfoMsg>::SharedPtr r_info_publisher;

            VstkConfig config;
            DiskIO io;
            std::vector<std::string> l_files;
            std::vector<std::string> r_files;
            int file_idx = 0;
            float time_period;
            
            inline sensor_msgs::msg::CameraInfo build_camera_info(cv::Mat image, cv::Mat K, cv::Mat d) {
                sensor_msgs::msg::CameraInfo cam_info = sensor_msgs::msg::CameraInfo();
                cam_info.distortion_model = "plumb_bob";
                cam_info.d.clear();
                for(int i=0; i<d.cols; i++) cam_info.d.push_back(d.at<double>(0, i));
                cam_info.height = image.rows;
                cam_info.width = image.cols;

                int idx = 0;
                for(int r=0; r < K.rows; r++) {
                    for(int c=0; c < K.cols; c++) {
                        cam_info.k.at(idx) = K.at<double>(r, c);
                        idx++;
                    }
                }
                return cam_info;
            }

            // async function that acts as the callback fn, reads from message queue and publishes anything new.
            void broadcast();
        public:
            explicit RosImageDirPublisher(VstkConfig config);
            void initialize();
    };


}

#endif
