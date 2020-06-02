from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
  return LaunchDescription([
    Node(
      parameters=[
            get_package_share_directory("orb_slam2_ros") + '/ros/config/params_t265_stereo.yaml'
      ],
      package='orb_slam2_ros',
      node_executable='orb_slam2_ros_stereo',
      name='orb_slam2_stereo',
      output='screen',
      remappings=[
                ('/camera/fisheye1/image_raw', '/cam_front/left/image_rect_color'),
                ('/camera/fisheye2/image_raw', '/cam_front/right/image_rect_color'),
      ]
    )
  ])

