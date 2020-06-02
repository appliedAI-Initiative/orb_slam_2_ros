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

#include "MonoNode.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<MonoNode>("mono", options);

  node->init();

  if (argc > 1) {
    RCLCPP_WARN(node->get_logger(), "Arguments supplied via command line are neglected.");
  }

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}

MonoNode::MonoNode(
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: Node(node_name, node_options)
{
  declare_parameter("image_topic", rclcpp::ParameterValue(std::string("/camera/image_raw")));
  declare_parameter("camera_info_topic",
    rclcpp::ParameterValue(std::string("/camera/camera_info")));
}

void MonoNode::init()
{
  Node::init(ORB_SLAM2::System::MONOCULAR);

  get_parameter("image_topic", image_topic_);
  get_parameter("camera_info_topic", camera_info_topic_);

  image_subscriber_ = image_transport_->subscribe(
    image_topic_, 1, &MonoNode::ImageCallback, this);
}

MonoNode::~MonoNode()
{
}

void MonoNode::ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  cv_bridge::CvImageConstPtr cv_in_ptr;
  try {
    cv_in_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  current_frame_time_ = msg->header.stamp;

  rclcpp::Time msg_time = cv_in_ptr->header.stamp;
  orb_slam_->TrackMonocular(cv_in_ptr->image, msg_time.seconds());

  Update();
}
