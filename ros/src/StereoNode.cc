#include "StereoNode.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions();
  auto node = rclcpp::Node::make_shared("Stereo");

  if(argc > 1) {
    RCLCPP_WARN(node->get_logger(), "Arguments supplied via command line are neglected.");
  }

  auto image_transport = std::make_shared<image_transport::ImageTransport>(node);

  // initialize
  StereoNode stereo_node(ORB_SLAM2::System::STEREO, node, image_transport);

  rclcpp::spin(node->get_node_base_interface());

  return 0;
}

StereoNode::StereoNode(const ORB_SLAM2::System::eSensor sensor,
                       rclcpp::Node::SharedPtr & node,
                       std::shared_ptr<image_transport::ImageTransport> & image_transport)
: Node (sensor, node, image_transport)
{
  left_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(node_, "image_left/image_color_rect");
  right_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(node_, "image_right/image_color_rect");
  camera_info_topic_ = "image_left/camera_info";

  sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *left_sub_, *right_sub_);
  sync_->registerCallback(std::bind(&StereoNode::ImageCallback, this,
    std::placeholders::_1, std::placeholders::_2));
}


StereoNode::~StereoNode () {
  delete sync_;
}


void StereoNode::ImageCallback (
  const sensor_msgs::msg::Image::ConstSharedPtr & msgLeft,
  const sensor_msgs::msg::Image::ConstSharedPtr & msgRight) {
  cv_bridge::CvImageConstPtr cv_ptrLeft;
  try {
    cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptrRight;
  try {
    cv_ptrRight = cv_bridge::toCvShare(msgRight);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  current_frame_time_ = msgLeft->header.stamp;

  rclcpp::Time msg_time = cv_ptrLeft->header.stamp;
  orb_slam_->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, msg_time.seconds());

  Update();
}
