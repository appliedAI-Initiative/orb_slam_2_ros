#include "RGBDNode.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions();
  auto node = rclcpp::Node::make_shared("RGBD");

  if(argc > 1) {
    RCLCPP_WARN(node->get_logger(), "Arguments supplied via command line are neglected.");
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  auto image_transport = std::make_shared<image_transport::ImageTransport>(node);

  RGBDNode rgbd_node(ORB_SLAM2::System::RGBD, node, image_transport);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}

RGBDNode::RGBDNode(const ORB_SLAM2::System::eSensor sensor,
                   rclcpp::Node::SharedPtr & node,
                   std::shared_ptr<image_transport::ImageTransport> & image_transport)
: Node(sensor, node, image_transport) {
  rgb_subscriber_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(node_, "/camera/rgb/image_raw");
  depth_subscriber_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(node_, "/camera/depth_registered/image_raw");
  camera_info_topic_ = "/camera/rgb/camera_info";

  sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *rgb_subscriber_, *depth_subscriber_);
  sync_->registerCallback(std::bind(&RGBDNode::ImageCallback, this, std::placeholders::_1, std::placeholders::_2));
}


RGBDNode::~RGBDNode () {
  delete sync_;
}

void RGBDNode::ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msgRGB,
                             const sensor_msgs::msg::Image::ConstSharedPtr &msgD)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try {
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msgRGB->header.stamp;

  rclcpp::Time msg_time = cv_ptrRGB->header.stamp;
  orb_slam_->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image, msg_time.seconds());

  Update();
}
