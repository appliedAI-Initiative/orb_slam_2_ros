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

#ifndef STEREONODE_HPP_
#define STEREONODE_HPP_

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

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <memory>

#include "System.h"

#include "Node.hpp"

class StereoNode : public Node
{
public:
  StereoNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & node_options);

  ~StereoNode();

  void init();

  void ImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & msgLeft,
    const sensor_msgs::msg::Image::ConstSharedPtr & msgRight);

private:
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
      sensor_msgs::msg::Image> sync_pol;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub_;
  message_filters::Synchronizer<sync_pol> * sync_;

  int resize_horizontal;
  int resize_vertical;
};

#endif  // STEREONODE_HPP_
