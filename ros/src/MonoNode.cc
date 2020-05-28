#include "MonoNode.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  auto options = rclcpp::NodeOptions();
  auto node = rclcpp::Node::make_shared("Mono");

  if(argc > 1) {
    RCLCPP_WARN(node->get_logger(), "Arguments supplied via command line are neglected.");
  }

  auto image_transport = std::make_shared<image_transport::ImageTransport>(node);

  MonoNode mono_node(ORB_SLAM2::System::MONOCULAR, node, image_transport);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}


MonoNode::MonoNode(
  ORB_SLAM2::System::eSensor sensor,
  rclcpp::Node::SharedPtr & node,
  std::shared_ptr<image_transport::ImageTransport> & image_transport)
: Node (sensor, node, image_transport) {
  image_subscriber_ = image_transport->subscribe("/camera/image_raw", 1, &MonoNode::ImageCallback, this);
  camera_info_topic_ = "/camera/camera_info";
}


MonoNode::~MonoNode () {
}


void MonoNode::ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
  cv_bridge::CvImageConstPtr cv_in_ptr;
  try {
      cv_in_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msg->header.stamp;

  rclcpp::Time msg_time = cv_in_ptr->header.stamp;
  orb_slam_->TrackMonocular(cv_in_ptr->image, msg_time.seconds());

  Update();
}
