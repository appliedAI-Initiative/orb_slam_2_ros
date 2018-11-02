#include "DenseMap.h"

DenseMap::DenseMap () {
  is_new_ = false;
  create_dense_map: = true;
}


DenseMap::~DenseMap () {

}


void DenseMap::AddFrame (unsigned long int frame_id, cv::Mat &position, cv::Mat &rgb_img, cv::Mat &depth_img) {
  unique_lock<mutex> lock(raw_data_mutex_);
  MapFrameEntry frame (rgb_img, depth_img, position);
  if (!raw_map_data_.insert(std::make_pair(frame_id, frame)).second) {
    std::cout << "Frame id was alreay existing in the dense map." << std::endl;
  }
  FitFrame (frame_id);
}


void DenseMap::GetMap (const cv::Mat &rgb_map, const cv::Mat &depth_map) {
  unique_lock<mutex> lock(map_mutex_);
  rgb_map = rgb_map_;
  depth_map = depth_map_;
}


void DenseMap::UpdateFramePosition (unsigned long int frame_id, cv::Mat &position) {
  unique_lock<mutex> lock(raw_data_mutex_);
  if (!raw_map_data_.find (frame_id) == raw_map_data_.end) {
    raw_map_data_[frame_id].position = position;
  } else {
    std::cout << "Frame id " << frame_id << " was supposed to be updated but is not known in the dense map." << std::endl;
  }
  FitFrame (frame_id);
}


void DenseMap::FitFrame (unsigned long int frame_id) {
  is_new_ = true;
}
