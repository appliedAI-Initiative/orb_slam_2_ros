#pragma once

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/types.hpp>


#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core/core.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "System.h"
#include "Node.hpp"

class StereoNode : public Node
{
public:
  StereoNode(
    const ORB_SLAM2::System::eSensor sensor,
    rclcpp::Node::SharedPtr & node,
    std::shared_ptr<image_transport::ImageTransport> & image_transport);

  ~StereoNode();

  void ImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & msgLeft,
    const sensor_msgs::msg::Image::ConstSharedPtr & msgRight);

private:
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_pol;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub_;
  message_filters::Synchronizer<sync_pol> *sync_;

  int resize_horizontal;
  int resize_vertical;
};
