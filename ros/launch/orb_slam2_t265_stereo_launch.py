from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
  return LaunchDescription([
    Node(
      parameters=[
        os.path.join(get_package_share_directory("orb_slam2_ros"), 'ros', 'config', 'params_t265_stereo.yaml'),
        {"voc_file": os.path.join(get_package_share_directory("orb_slam2_ros") + '/orb_slam2/Vocabulary/ORBvoc.txt')},
      ],
      package='orb_slam2_ros',
      node_executable='orb_slam2_ros_stereo',
      node_name='orb_slam2_ros_stereo',
      output='screen',
      remappings=[
        ('/image_left/image_color_rect', '/camera/fisheye1/image_raw'),
        ('/image_right/image_color_rect', '/camera/fisheye2/image_raw'),
        ('/camera/camera_info', '/camera/fisheye1/camera_info'),
      ]
    )
  ])
