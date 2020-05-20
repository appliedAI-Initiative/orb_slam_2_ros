#include "RGBDNode.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if(argc > 1) {
        RCLCPP_WARN(this->get_logger(), "Arguments supplied via command line are neglected.");
    }

    auto node = rclcpp::Node::make_shared("RGBD");

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    image_transport::ImageTransport image_transport (node);

    RGBDNode node (ORB_SLAM2::System::RGBD, node, image_transport);

    node.Init();

    rclcpp::spin(std::make_shared<Node>());

    rclcpp::shutdown();

    return 0;
}


RGBDNode::RGBDNode (const ORB_SLAM2::System::eSensor sensor, auto node = rclcpp::Node::make_shared("RGBD"), image_transport::ImageTransport &image_transport) : Node (sensor, node, image_transport) {
  rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::msg::Image> (node, "/camera/rgb/image_raw", 1);
  depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::msg::Image> (node, "/camera/depth_registered/image_raw", 1);
  camera_info_topic_ = "/camera/rgb/camera_info";

  sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *rgb_subscriber_, *depth_subscriber_);
  sync_->registerCallback(boost::bind(&RGBDNode::ImageCallback, this, _1, _2));
}


RGBDNode::~RGBDNode () {
  delete rgb_subscriber_;
  delete depth_subscriber_;
  delete sync_;
}


void RGBDNode::ImageCallback (const sensor_msgs::msg::ImageConstPtr& msgRGB, const sensor_msgs::msg::ImageConstPtr& msgD) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try {
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msgRGB->header.stamp;

  orb_slam_->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

  Update ();
}
