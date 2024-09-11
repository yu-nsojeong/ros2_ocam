import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
        package='ocam',
        executable='ocam',
        name='ocam',
        output='screen',
        parameters=[
          {'resolution': 2},  # 0: 1280x960, 1: 1280x720, 2: 640x480, 3: 640x360
          {'frame_rate': 15.0},
          {'exposure': 150},
          {'gain': 30},
          {'wb_blue': 40},
          {'wb_red': 220},
          {'auto_exposure': False},
          {'show_image': True},
          ]
      ),
  ])
