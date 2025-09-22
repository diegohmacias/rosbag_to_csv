from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbag_to_csv',
            executable='rosbag_to_csv_gui',  # console_script name from setup.py
            name='rosbag_to_csv_gui',
            output='screen',
            emulate_tty=True,
        )
    ])
