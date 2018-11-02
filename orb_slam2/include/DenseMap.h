/**
* This file is part of ORB-SLAM2.
*
* Lennart Haller, 2018
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef DENSE_MAP_H_
#define DENSE_MAP_H_

#include "KeyFrame.h"

#include <mutex>
#include <thread>
#include <map>

namespace ORB_SLAM2
{

  class DenseMap
{
  public:
    DenseMap ();
    DenseMap ();
    void AddFrame (unsigned long int frame_id, cv::Mat &position, cv::Mat &rgb_img, cv::Mat &depth_img);
    bool IsNew () {return is_new_;}
    void GetMap (const cv::Mat &rgb_map, const cv::Mat &depth_map);
    void UpdateFramePosition (unsigned long int frame_id, cv::Mat &position);

  private:
    void FitFrame (unsigned long int frame_id);
    bool is_new_;
    std::map <unsigned long int, MapImageEntry> raw_map_data_;
    cv::Mat rgb_map_;
    cv::Mat depth_map_;

    std::mutex map_mutex_;
    std::mutex raw_data_mutex_;

    struct MapImageEntry {
      MapFrameEntry (cv::Mat rgb, cv::Mat depth, cv::Mat pos)
      : rgb_img(rgb), depth_img (depth) position (pos) {}
      cv::Mat rgb_img;
      cv::Mat depth_img;
      cv::Mat position;
    };

};

}// namespace ORB_SLAM

#endif // DENSE_MAP_H_
