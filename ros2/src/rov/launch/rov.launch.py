import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'rov'

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )])
    )

    serial_node = Node(
        package='serial',
        executable='bidirectional',
        name='serial_node',
        parameters=[
            {'port': '/dev/ttyACM2'},
        ],
    )

    return LaunchDescription([
        joystick,
        serial_node,
    ])

