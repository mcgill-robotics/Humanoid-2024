import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='controls',
        #     executable='controls',
        #     name='controls',
        #     output='screen'
        # ),
        ExecuteProcess(
            cmd=[['ros2 launch phidgets_spatial spatial-launch.py']],
            shell=True
        )
    ])
