from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Include the robot controllers launch file
    controllers_launch = os.path.join(
        get_package_share_directory('lerobot_description'),
        'launch',
        'robot_controllers.launch.py'
    )

    return LaunchDescription([
        # Include the robot controllers (this sets up the topic_based_ros2_control plugin)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(controllers_launch)
        ),

        # Start the joystick controller
        Node(
            package='joystick_controller',
            executable='joystick_controller_node',  # Updated to match setup.py entry point
            name='joystick_controller',
            output='screen'
        ),

        # Start the keyboard joystick (optional - for testing without physical joystick)
        Node(
            package='joystick_controller',
            executable='keyboard_joystick',  # This one is already correct
            name='keyboard_joystick',
            output='screen'
        )
    ])