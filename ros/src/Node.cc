#include "Node.h"

Node::Node () {

}


Node::~Node () {

}


tf::Transform Node::TransformFromMat (cv::Mat position_mat) {
  cv::Mat rotation (3,3,CV_32F);
  cv::Mat translation (3,1,CV_32F);
  rotation = position_mat.rowRange(0,3).colRange(0,3).t();
  translation = -rotation*position_mat.rowRange(0,3).col(3);

  tf::Matrix3x3 ros_rotation_mat (rotation.at<float> (0,0), rotation.at<float> (0,1), rotation.at<float> (0,2),
                                  rotation.at<float> (1,0), rotation.at<float> (1,1), rotation.at<float> (1,2),
                                  rotation.at<float> (2,0), rotation.at<float> (2,1), rotation.at<float> (2,2));
  tf::Vector3 ros_translation_vec (translation.at<float> (0,0), translation.at<float> (0,1), translation.at<float> (0,2));

  return tf::Transform (ros_rotation_mat, ros_translation_vec);
}
