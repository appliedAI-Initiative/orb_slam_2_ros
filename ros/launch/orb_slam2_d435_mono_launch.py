from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
  return LaunchDescription([
    Node(
      parameters=[
        os.path.join(get_package_share_directory("orb_slam2_ros"), 'ros', 'config', 'params_d435_mono.yaml'),
        {"voc_file": os.path.join(get_package_share_directory("orb_slam2_ros") + '/orb_slam2/Vocabulary/ORBvoc.txt')},
      ],
      package='orb_slam2_ros',
      node_executable='orb_slam2_ros_mono',
      node_name='orb_slam2_ros_mono',
      output='screen',
      remappings=[
        ('/camera/image_raw', '/camera/color/image_raw'),
        ('/camera/camera_info', '/camera/color/camera_info'),
      ]
    )
  ])
