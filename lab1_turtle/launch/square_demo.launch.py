import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory('lab1_turtle')
    system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'system.launch.py')))
    send_square = TimerAction(period=4.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'action', 'send_goal', '--feedback', '/draw_square',
                 'lab1_interfaces/action/DrawSquare', '{side_length: 3.0}'],
            output='screen')])
    return LaunchDescription([system, send_square])
