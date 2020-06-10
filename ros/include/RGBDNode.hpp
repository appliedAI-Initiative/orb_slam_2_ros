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

#ifndef RGBDNODE_HPP_
#define RGBDNODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/core/core.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <memory>

#include "System.h"

#include "Node.hpp"

class RGBDNode : public Node
{
public:
  RGBDNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & node_options);

  ~RGBDNode();

  void init();

  void ImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & msgRGB,
    const sensor_msgs::msg::Image::ConstSharedPtr & msgD);

private:
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
      sensor_msgs::msg::Image> sync_pol;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_subscriber_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_subscriber_;
  message_filters::Synchronizer<sync_pol> * sync_;
};

#endif  // RGBDNODE_HPP_
