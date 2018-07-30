#include "Node.h"

#include <iostream>

Node::Node () {

}


Node::~Node () {

}


void Node::Launch (ORB_SLAM2::System* pSLAM, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) {
  num_of_pointclouds_published_ = 0;
  orb_slam_ = pSLAM;

  rendered_image_publisher_ = image_transport.advertise ("/orbslam2/debug_image", 1);
  map_points_publisher_ = node_handle.advertise<sensor_msgs::PointCloud2> ("/orbslam2/map_points", 1);
}


tf::Transform Node::TransformFromMat (cv::Mat position_mat) {
  cv::Mat rotation(3,3,CV_32F);
  cv::Mat translation(3,1,CV_32F);

  rotation = position_mat.rowRange(0,3).colRange(0,3).t();
  translation = rotation*position_mat.rowRange(0,3).col(3);

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

  return tf::Transform (tf_camera_rotation, tf_camera_translation);
}


void Node::PublishMapPoints (std::vector<ORB_SLAM2::MapPoint*> map_points) {
  sensor_msgs::PointCloud2 cloud = MapPointsToPointCloud (map_points);
  map_points_publisher_.publish (cloud);
}

sensor_msgs::PointCloud2 Node::MapPointsToPointCloud (std::vector<ORB_SLAM2::MapPoint*> map_points) {

  if (map_points.size() == 0) {
    std::cout << "Map point vector is empty!" << std::endl;
  }

  sensor_msgs::PointCloud2 cloud;

  const int num_channels = 3; // x y z

  cloud.header.stamp = ros::Time::now(); //WARNING: TODO: Timestamp needs to come from the curent image not current time otherwise it lags behind some msecs
  cloud.header.frame_id = "map"; //TODO: read from config
  cloud.header.seq = num_of_pointclouds_published_;
  cloud.height = 1;
  cloud.width = map_points.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = { "x", "y", "z"};
  for (int i = 0; i<num_channels; i++) {
  	cloud.fields[i].name = channel_id[i];
  	cloud.fields[i].offset = i * sizeof(float);
  	cloud.fields[i].count = 1;
  	cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);

	unsigned char *cloud_data_ptr = &(cloud.data[0]);

  float data_array[3];
  for (unsigned int i=0; i<cloud.width; i++) {
    if (!map_points.at(i)->isBad()) {
      data_array[0] = -1.0* map_points.at(i)->GetWorldPos().at<float> (2); //x. Do the transformation by just reading at the position of z instead of x
    	data_array[1] = map_points.at(i)->GetWorldPos().at<float> (0); //y. Do the transformation by just reading at the position of x instead of y
    	data_array[2] = -1.0* map_points.at(i)->GetWorldPos().at<float> (1); //z. Do the transformation by just reading at the position of y instead of z
      //TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and PublishMapPoints

      memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, 3*sizeof(float));
    }
  }

  num_of_pointclouds_published_ ++;

  return cloud;
}
