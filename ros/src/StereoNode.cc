#include "StereoNode.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if(argc > 1) {
        RCLCPP_WARN(this->get_logger(), "Arguments supplied via command line are neglected.");
    }

    auto node = rclcpp::Node::make_shared("Stereo");
    image_transport::ImageTransport image_transport (node);

    // initialize
    StereoNode node (ORB_SLAM2::System::STEREO, node, image_transport);

    node.Init();

    rclcpp::spin(std::make_shared<Node>());

    return 0;
}


StereoNode::StereoNode (const ORB_SLAM2::System::eSensor sensor, auto node = rclcpp::Node::make_shared("Stereo"), image_transport::ImageTransport &image_transport) : Node (sensor, node, image_transport) {
    left_sub_ = new message_filters::Subscriber<sensor_msgs::msg::Image> (node, "image_left/image_color_rect", 1);
    right_sub_ = new message_filters::Subscriber<sensor_msgs::msg::Image> (node, "image_right/image_color_rect", 1);
    camera_info_topic_ = "image_left/camera_info";

    sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *left_sub_, *right_sub_);
    sync_->registerCallback(boost::bind(&StereoNode::ImageCallback, this, _1, _2));
}


StereoNode::~StereoNode () {
    delete left_sub_;
    delete right_sub_;
    delete sync_;
}


void StereoNode::ImageCallback (const sensor_msgs::msg::ImageConstSharedPtr& msgLeft, const sensor_msgs::msg::ImageConstSharedPtr& msgRight) {
  cv_bridge::CvImageConstPtr cv_ptrLeft;
  try {
      cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
  } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrRight;
  try {
      cv_ptrRight = cv_bridge::toCvShare(msgRight);
  } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msgLeft->header.stamp;

  orb_slam_->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, cv_ptrLeft->header.stamp.toSec());

  Update ();
}
