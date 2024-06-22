#include "io/ros/RosPublisher.hpp"

using namespace vstk;
using namespace std::chrono_literals;

RosImageDirPublisher::RosImageDirPublisher(VstkConfig config) : 
  Node("vstk_image_pub"),
  config(config), 
  frame_rate(30) 
{
  RosConfig ros_conf = this->config.get_ros_config();
  this->l_publisher = this->create_publisher<sensor_msgs::msg::Image>(ros_conf.left_image_topic, 10);
  this->r_publisher = this->create_publisher<sensor_msgs::msg::Image>(ros_conf.right_image_topic, 10);
  this->l_info_publisher = this->create_publisher<sensor_msgs::msg::CameraInfo>(ros_conf.left_cam_info_topic, 10);
  this->r_info_publisher = this->create_publisher<sensor_msgs::msg::CameraInfo>( ros_conf.right_cam_info_topic, 10);
  
  this->timer_ = this->create_wall_timer(500ms, std::bind(&RosImageDirPublisher::broadcast, this));
  this->time_period = (1 / this->frame_rate);
}

void RosImageDirPublisher::initialize() {
  DBGLOG("Initializing ROS publisher 'vstk_image_pub'...");
  this->l_files = this->io.list_directory(this->config.get_stereo_src_1());
  if(this->l_files.size() == 0) {
    ERRORLOG("Failed to list directory pattern %s, no files found!", this->config.get_stereo_src_1());
    exit(-1);
  }

  this->r_files = this->io.list_directory(this->config.get_stereo_src_2());
  if(this->r_files.size() == 0) {
    ERRORLOG("Failed to list directory pattern %s, no files found!", this->config.get_stereo_src_2());
    exit(-1);
  }
  DBGLOG("Found %ld files with pattern %s", this->l_files.size(), this->config.get_stereo_src_1());
  DBGLOG("Publisher 'vstk_image_pub' ready for execution");
}

void RosImageDirPublisher::broadcast() {
  if(file_idx >= this->l_files.size()) {
    RCLCPP_INFO(this->get_logger(), "Completed broadcast of all images!, press [ctrl + c] to end process");
    return;
  }

  cv::Mat l_image = cv::imread(this->l_files[file_idx], cv::IMREAD_COLOR);
  cv::Mat r_image = cv::imread(this->r_files[file_idx], cv::IMREAD_COLOR);

  StereoCamParamsPtr stereo_params = this->config.get_stereo_cam_params();

  sensor_msgs::msg::CameraInfo l_info = build_camera_info(l_image, stereo_params->cam1_params.K, stereo_params->cam1_params.dist_coeff);
  sensor_msgs::msg::CameraInfo r_info = build_camera_info(r_image, stereo_params->cam2_params.K, stereo_params->cam2_params.dist_coeff);
  
  std_msgs::msg::Header header = std_msgs::msg::Header();
  header.frame_id = this->l_files[file_idx];

  sensor_msgs::msg::Image::SharedPtr l_msg = cv_bridge::CvImage(
    header, 
    "bgr8", 
    l_image
  ).toImageMsg();

  sensor_msgs::msg::Image::SharedPtr r_msg = cv_bridge::CvImage(
    header,
    "bgr8",
    r_image
  ).toImageMsg();

  l_info.header = header;
  r_info.header = header;

  this->l_publisher->publish(*l_msg);
  this->r_publisher->publish(*r_msg);
  this->l_info_publisher->publish(l_info);
  this->r_info_publisher->publish(r_info);

  RCLCPP_INFO(this->get_logger(), "Published image pair"); 
  this->file_idx++;
}


VstkCore::VstkCore(VstkConfig conf) : Node("vstk_core") , conf(conf) {
  // this->pcl_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(conf.get_ros_config().p3d_topic, 10);
  // this->image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(conf.get_ros_config().left_image_topic);
}

void VstkCore::initialize() {
  this->conf = conf;
}



