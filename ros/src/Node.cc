#include "Node.h"

Node::Node () {
  previous_pose_ = cv::Mat::eye(4,4, CV_32F);
}


Node::~Node () {

}


tf::Transform Node::TransformFromMat (cv::Mat position_mat) {
  cv::Mat world_left_hand = cv::Mat::eye(4,4, CV_32F);
  // matrix to flip signs of sinus in rotation matrix
  const cv::Mat flip_sign = (cv::Mat_<float>(4,4) <<   1,-1,-1, 1,
                                                      -1, 1,-1, 1,
                                                      -1,-1, 1, 1,
                                                      1, 1, 1, 1);

  cv::Mat translation =  (position_mat * previous_pose_.inv()).mul(flip_sign);
  world_left_hand = world_left_hand * translation;
  previous_pose_ =  position_mat.clone();

  tf::Matrix3x3 camera_rotation_right_hand (world_left_hand.at<float> (0,0), world_left_hand.at<float> (0,1), world_left_hand.at<float> (0,2),
                                            world_left_hand.at<float> (1,0), world_left_hand.at<float> (1,1), world_left_hand.at<float> (1,2),
                                            world_left_hand.at<float> (2,0), world_left_hand.at<float> (2,1), world_left_hand.at<float> (2,2));
  tf::Vector3 camera_translation_right_hand (world_left_hand.at<float> (0,3), world_left_hand.at<float> (1,3), world_left_hand.at<float> (2,3));

  //rotate 270deg about x and 270deg about x to get ENU: x forward, y left, z up
  const tf::Matrix3x3 rotation_270_deg_xz(0, 1, 0,
                                          0, 0, 1,
                                          1, 0, 0);

  tf::Matrix3x3 global_rotation_right_hand = camera_rotation_right_hand * rotation_270_deg_xz;
  tf::Vector3 global_translation_right_hand = camera_translation_right_hand * rotation_270_deg_xz;

  return tf::Transform (global_rotation_right_hand, global_translation_right_hand);
}
