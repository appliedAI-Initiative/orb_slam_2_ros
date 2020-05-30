/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "RGBDNode.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<RGBDNode>("RGBD", options);

  if (argc > 1) {
    RCLCPP_WARN(node->get_logger(), "Arguments supplied via command line are neglected.");
  }

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}

RGBDNode::RGBDNode(
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: Node(node_name, node_options)
{
  Node::init(ORB_SLAM2::System::RGBD);

  declare_parameter("rgb_image_topic",
    rclcpp::ParameterValue(std::string("/camera/rgb/image_raw")));
  declare_parameter("depth_image_topic",
    rclcpp::ParameterValue(std::string("/camera/depth_registered/image_raw")));
  declare_parameter("camera_info_topic",
    rclcpp::ParameterValue(std::string("/camera/rgb/camera_info")));

  get_parameter("rgb_image_topic", rgb_image_topic_);
  get_parameter("depth_image_topic", depth_image_topic_);
  get_parameter("camera_info_topic", camera_info_topic_);

  rgb_subscriber_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
    shared_from_this(), rgb_image_topic_);
  depth_subscriber_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
    shared_from_this(), depth_image_topic_);

  sync_ = new message_filters::Synchronizer<sync_pol>(
    sync_pol(10), *rgb_subscriber_, *depth_subscriber_);
  sync_->registerCallback(
    std::bind(&RGBDNode::ImageCallback, this, std::placeholders::_1, std::placeholders::_2));
}

RGBDNode::~RGBDNode()
{
  delete sync_;
}

void RGBDNode::ImageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & msgRGB,
  const sensor_msgs::msg::Image::ConstSharedPtr & msgD)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try {
    cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  current_frame_time_ = msgRGB->header.stamp;

  rclcpp::Time msg_time = cv_ptrRGB->header.stamp;
  orb_slam_->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, msg_time.seconds());

  Update();
}
