#include "Node.h"

Node::Node () {
  previous_pose_ = cv::Mat::eye(4,4, CV_32F);
}


Node::~Node () {

}


tf::Transform Node::TransformFromMat (cv::Mat position_mat) {
  cv::Mat rotation(3,3,CV_32F);
  cv::Mat translation(3,1,CV_32F);

  rotation = position_mat.rowRange(0,3).colRange(0,3).t();
  translation = position_mat.rowRange(0,3).col(3);

  tf::Matrix3x3 tf_camera_rotation (rotation.at<float> (0,0), rotation.at<float> (0,1), rotation.at<float> (0,2), //Z
                                            rotation.at<float> (1,0), rotation.at<float> (1,1), rotation.at<float> (1,2), //Y
                                            rotation.at<float> (2,0), rotation.at<float> (2,1), rotation.at<float> (2,2)  //X
                                          );

  tf::Vector3 tf_camera_translation (translation.at<float> (0), translation.at<float> (1), translation.at<float> (2));

  const tf::Matrix3x3 Rx (1, 0, 0,
                          0, 0, -1,
                          0, 1, 0);

  const tf::Matrix3x3 Rz (0, -1, 0,
                          1, 0, 0,
                          0, 0, 1);

                          tf_camera_rotation = Rx*tf_camera_rotation;
                          tf_camera_rotation = Rz*tf_camera_rotation;
                          tf_camera_translation = Rx*tf_camera_translation;
                          tf_camera_translation = Rz*tf_camera_translation;

  tf_camera_translation = tf_camera_rotation*tf_camera_translation;

  return tf::Transform (tf_camera_rotation, tf_camera_translation);
}
