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

#ifndef ORBSLAM2_ROS_NODE_H_
#define ORBSLAM2_ROS_NODE_H_

#include <vector>
#include <ros/ros.h>
#include <ros/time.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include "System.h"



class Node
{
  public:
    Node (ORB_SLAM2::System* pSLAM, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport);
    ~Node ();

  protected:
    tf::Transform TransformFromMat (cv::Mat position_mat);
    void PublishMapPoints (std::vector<ORB_SLAM2::MapPoint*> map_points);
    void UpdateParameters ();
    ORB_SLAM2::System* orb_slam_;
    image_transport::Publisher rendered_image_publisher_;
    ros::Publisher map_points_publisher_;

    bool publish_pointcloud_param_;
    bool localize_only_param_;
    std::string map_frame_id_param_;
    std::string camera_frame_id_param_;

  private:
    sensor_msgs::PointCloud2 MapPointsToPointCloud (std::vector<ORB_SLAM2::MapPoint*> map_points);
    void InitParameters ();

    std::string name_of_node_;
    int num_of_pointclouds_published_;
    ros::NodeHandle node_handle_;
};

#endif //ORBSLAM2_ROS_NODE_H_
