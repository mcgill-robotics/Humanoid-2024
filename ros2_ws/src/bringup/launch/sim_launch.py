import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controls',
            executable='controls',
            name='controls',
            output='screen',
            parameters=[{"ROS_IP": "0.0.0.0"}, {"ROS_TCP_PORT": 10000}],
        ),
        ExecuteProcess(
            cmd=[['ros2 launch sim endpoint.py']],
            shell=True
        )
    ])
