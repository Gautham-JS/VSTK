#ifndef __VSTK_ROS_PIPELINE_H
#define __VSTK_ROS_PIPELINE_H

#include <stdio.h>
#include <vector>
#include <string>
#include <thread>
#include <mutex>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.h"

#include "features/FeatureExtractor.hpp"
#include "utils/Logger.hpp"
#include "utils/CvUtils.hpp"
#include "io/ros/RosDefs.hpp"


typedef sensor_msgs::msg::Image         RosImageMsg;
typedef sensor_msgs::msg::CameraInfo    RosCameraInfoMsg;
typedef sensor_msgs::msg::PointCloud2   RosPCL2Msg;

namespace vstk {
    class VstkCore : public rclcpp::Node {
        private:
            uint8_t frame_rate;
            std::string pcl_topic;

            std::mutex mtx;


            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<RosPCL2Msg>::SharedPtr pcl_publisher;
            
            message_filters::Subscriber<RosImageMsg> l_img_subscriber;
            message_filters::Subscriber<RosCameraInfoMsg> l_info_subscriber;
            message_filters::Subscriber<RosImageMsg> r_img_subscriber;
            message_filters::Subscriber<RosCameraInfoMsg> r_info_subscriber;

            std::shared_ptr<
                message_filters::TimeSynchronizer<
                    RosImageMsg,
                    RosCameraInfoMsg,
                    RosImageMsg,
                    RosCameraInfoMsg 
                >
            > syncer;

            std::shared_mutex mutex;
            RosImageMsg ref_image;
            RosCameraInfoMsg ref_info;
        
            VstkConfig conf;            

            // async function that acts as the callback fn.
            void timer_callback(
                const RosImageMsg::ConstSharedPtr &l_image,
                const RosCameraInfoMsg::ConstSharedPtr &l_cam_info,
                const RosImageMsg::ConstSharedPtr &r_image,
                const RosCameraInfoMsg::ConstSharedPtr &r_cam_info
            );

        public:
            explicit VstkCore(VstkConfig conf);
    };
}

#endif