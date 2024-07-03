#include "io/ros/RosPipeline.hpp"

using namespace vstk;


void msg_callback(
    const RosImageMsg::ConstSharedPtr &l_image,
    const RosCameraInfoMsg::ConstSharedPtr &l_cam_info,
    const RosImageMsg::ConstSharedPtr &r_image,
    const RosCameraInfoMsg::ConstSharedPtr &r_cam_info
) {
    INFOLOG( "Received message group");
    INFOLOG( "---MESSAGE INFO START---");
    INFOLOG( "\t- l_image ID : %s", l_image->header.frame_id.c_str());
    INFOLOG( "\t- r_image ID : %s", r_image->header.frame_id.c_str());
    INFOLOG( "\t- l_cam_info ID : %s", l_cam_info->header.frame_id.c_str());
    INFOLOG( "\t- r_cam_info ID : %s", r_cam_info->header.frame_id.c_str());
    INFOLOG( "---MESSAGE INFO END---");
}

VstkCore::VstkCore(VstkConfig conf) : Node("vstk_core") , conf(conf) {
    this->pcl_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(conf.get_ros_config().p3d_topic, 10);
  
    this->l_img_subscriber.subscribe(this, conf.get_ros_config().left_image_topic);
    this->l_info_subscriber.subscribe(this, conf.get_ros_config().left_cam_info_topic);

    this->r_img_subscriber.subscribe(this, conf.get_ros_config().right_image_topic);
    this->r_info_subscriber.subscribe(this, conf.get_ros_config().right_cam_info_topic);
    
    this->syncer = std::make_shared<message_filters::TimeSynchronizer<RosImageMsg, RosCameraInfoMsg, RosImageMsg, RosCameraInfoMsg>>(
        l_img_subscriber,
        l_info_subscriber,
        r_img_subscriber,
        r_info_subscriber,
        10
    );
    this->syncer->registerCallback(msg_callback);
} 


void VstkCore::timer_callback(
    const RosImageMsg::ConstSharedPtr &l_image,
    const RosCameraInfoMsg::ConstSharedPtr &l_cam_info,
    const RosImageMsg::ConstSharedPtr &r_image,
    const RosCameraInfoMsg::ConstSharedPtr &r_cam_info
) {
    
    
}