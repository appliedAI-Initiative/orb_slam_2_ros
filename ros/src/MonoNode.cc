#include "MonoNode.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if(argc > 1) {
        RCLCPP_WARN(this->get_logger(), "Arguments supplied via command line are neglected.");
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    auto node = rclcpp::Node::make_shared("Mono");
    image_transport::ImageTransport image_transport (node);

    MonoNode node (ORB_SLAM2::System::MONOCULAR, node, image_transport);

    node.Init();

    rclcpp::spin(std::make_shared<Node>());

    rclcpp::shutdown();

    return 0;
}


MonoNode::MonoNode (ORB_SLAM2::System::eSensor sensor, auto node = rclcpp::Node::make_shared("Mono"), image_transport::ImageTransport &image_transport) : Node (sensor, node, image_transport) {
  image_subscriber = image_transport.subscribe ("/camera/image_raw", 1, &MonoNode::ImageCallback, this);
  camera_info_topic_ = "/camera/camera_info";
}


MonoNode::~MonoNode () {
}


void MonoNode::ImageCallback (const sensor_msgs::msg::ImageConstPtr& msg) {
  cv_bridge::CvImageConstPtr cv_in_ptr;
  try {
      cv_in_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msg->header.stamp;

  orb_slam_->TrackMonocular(cv_in_ptr->image,cv_in_ptr->header.stamp.toSec());

  Update ();
}
