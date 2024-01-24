import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hardware_interface',  # Replace with your package name
            executable='hardware_interface',   # Replace with your executable name
            name='hardware_interface',
            output='screen',
        ),
        ExecuteProcess(
            cmd=[['ros2 launch phidgets_spatial spatial-launch.py']],
            shell=True
        )
    ])
