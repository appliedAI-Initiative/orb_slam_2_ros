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

#include "StereoNode.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo");
    ros::start();

    if(argc != 3)
    {
        ROS_ERROR ("Path to vocabulary and path to settings need to be set.");
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO);
    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport (node_handle);

    message_filters::Subscriber<sensor_msgs::Image> left_sub(node_handle, "image_left/image_color_rect", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(node_handle, "image_right/image_color_rect", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);

    // initilaize
    StereoNode node (&SLAM, node_handle, image_transport);

    // register callbacks
    sync.registerCallback(boost::bind(&StereoNode::ImageCallback, &node,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}


StereoNode::StereoNode (ORB_SLAM2::System* pSLAM, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) : Node (pSLAM, node_handle, image_transport) {
}


StereoNode::~StereoNode () {
}


void StereoNode::ImageCallback (const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


  cv::Mat position = orb_slam_->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
  if (!position.empty()) {
    tf::Transform transform = TransformFromMat (position);
    static tf::TransformBroadcaster tf_broadcaster;
    tf_broadcaster.sendTransform(tf::StampedTransform(transform, cv_ptrLeft->header.stamp, map_frame_id_param_, camera_frame_id_param_));
  }

  const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(cv_ptrLeft->header, "bgr8", orb_slam_->DrawCurrentFrame()).toImageMsg();

  if (publish_pointcloud_param_) {
    PublishMapPoints (orb_slam_->GetAllMapPoints());
  }

  UpdateParameters ();

  rendered_image_publisher_.publish (rendered_image_msg);
}
