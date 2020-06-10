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

#ifndef MONONODE_HPP_
#define MONONODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core/core.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>

#include "System.h"

#include "Node.hpp"

class MonoNode : public Node
{
public:
  MonoNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & node_options);

  ~MonoNode();

  void init();

  void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

private:
  image_transport::Subscriber image_subscriber_;
};

#endif  // MONONODE_HPP_
