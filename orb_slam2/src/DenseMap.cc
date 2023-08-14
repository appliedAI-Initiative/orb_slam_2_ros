#include "DenseMap.h"

namespace ORB_SLAM2
{

DenseMap::DenseMap () {
  is_new_ = false;
  create_dense_map_ = true;
  current_task_id_ = 0;
  /*
   * Constructing a TaskQueue object. Not sure yet how and where we use it.
   */
  task_queue_ = new TaskQueue::TaskQueue<DenseMap::PointCloudRGBD::Ptr, DenseMap::PointCloudRGBD::Ptr, DenseMap::PointCloudRGBD::Ptr, cv::Mat> (1);
}


DenseMap::~DenseMap () {
  delete task_queue_;
}


void DenseMap::AddFrame (const unsigned long int frame_id, const cv::Mat &position, const cv::Mat &rgb_img, const cv::Mat &depth_img) {
  unique_lock<mutex> lock(raw_data_mutex_);
  MapImageEntry frame (rgb_img.clone(), depth_img.clone(), position.clone());
  if (!raw_map_data_.insert(std::make_pair(frame_id, frame)).second) {
    std::cout << "Frame id was alreay existing in the dense map." << std::endl;
  }
  AddFrameToMap (frame_id);
}


void DenseMap::GetMap (cv::Mat &rgb_map, cv::Mat &depth_map) {
  unique_lock<mutex> lock(map_mutex_);
  rgb_map = rgb_map_;
  depth_map = depth_map_;
}


void DenseMap::UpdateFramePosition (unsigned long int frame_id, cv::Mat &position) {
  unique_lock<mutex> lock(raw_data_mutex_);
  auto it = raw_map_data_.find (frame_id);
  if (it != raw_map_data_.end()) {
    raw_map_data_[frame_id].position = position.clone();
  } else {
    std::cout << "Frame id " << frame_id << " was supposed to be updated but is not known in the dense map." << std::endl;
  }
  //TODO WARNING after all positions are updated the whole map needs to be recalculated
}


void DenseMap::AddFrameToMap (unsigned long int frame_id) {
  DenseMap::PointCloudRGBD::Ptr map (new DenseMap::PointCloudRGBD);
  MatsToPclRGBDCloud (depth_map_, rgb_map_, map);
  DenseMap::PointCloudRGBD::Ptr frame (new DenseMap::PointCloudRGBD);
  MatsToPclRGBDCloud (raw_map_data_[frame_id].depth_img, raw_map_data_[frame_id].rgb_img, frame);
  std::cout << "AE: AddFrameToMap 1 " << std::endl;
  while (task_queue_->NumJobsCurrentlyRunning () > 0) {
  } //Wait until current job has finished
  std::cout << "AE: AddFrameToMap 2 " << std::endl;
  //std::function<DenseMap::PointCloudRGBD::Ptr(DenseMap::PointCloudRGBD::Ptr, DenseMap::PointCloudRGBD::Ptr, cv::Mat)>
  //auto test = std::bind(&DenseMap::FitFrame, this, map, frame, raw_map_data_[frame_id].position);
  //DenseMap::FitFrame (map, frame, raw_map_data_[frame_id].position);  //&DenseMap::FitFrame;
  //task_queue_->AddTask (current_task_id_, 1, DenseMap::FitFrame);
  //task_queue_->AddTask (current_task_id_, 1, test);
  PclRGBDCloudToMats (map, depth_map_, rgb_map_);
  std::cout << "AE: AddFrameToMap 3 " << std::endl;
  is_new_ = true;
}


void DenseMap::MatsToPclRGBDCloud (cv::Mat depth_mat, cv::Mat rgb_mat, DenseMap::PointCloudRGBD::Ptr cloud) {
  for(int i=0; i<depth_mat.total(); i++) {
    PointRGBD point;
    point.x = depth_mat.at<float>(0,i);
    point.y = depth_mat.at<float>(1,i);
    point.z = depth_mat.at<float>(2,i);

    uint32_t rgb = (
      static_cast<uint32_t>(depth_mat.at<float>(2,i)) << 16 |
      static_cast<uint32_t>(depth_mat.at<float>(1,i)) << 8 |
      static_cast<uint32_t>(depth_mat.at<float>(0,i))
    );

    point.rgba = rgb;

    cloud->points.push_back(point);
  }
}


void DenseMap::PclRGBDCloudToMats (DenseMap::PointCloudRGBD::Ptr cloud, cv::Mat depth_mat, cv::Mat rgb_mat) {
  depth_mat = cv::Mat (3, cloud->points.size(), CV_64FC1);
  rgb_mat = cv::Mat (3, cloud->points.size(), CV_64FC1);

  for(int i=0; i < cloud->points.size();i++){
    depth_mat.at<double>(0,i) = cloud->points.at(i).x;
    depth_mat.at<double>(1,i) = cloud->points.at(i).y;
    depth_mat.at<double>(2,i) = cloud->points.at(i).z;

    Eigen::Vector3i rgb = cloud->points.at(i).getRGBVector3i();

    rgb_mat.at<double>(0,i) = rgb[2];
    rgb_mat.at<double>(0,i) = rgb[1];
    rgb_mat.at<double>(0,i) = rgb[0];
  }
}


DenseMap::PointCloudRGBD::Ptr DenseMap::FitFrame (DenseMap::PointCloudRGBD::Ptr map, DenseMap::PointCloudRGBD::Ptr frame, cv::Mat pos) {
  /*if (map is empty) { //TODO
    map = frame;
    return;
  }*/
  return map;
}

} // namespace ORB_SLAM
