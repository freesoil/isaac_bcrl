from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Joy node to read from joystick
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.05,
                'autorepeat_rate': 50.0,
            }]
        ),
        
        # Joystick controller node that publishes to ros2_control
        Node(
            package='joystick_controller',
            executable='joystick_controller_node',  # Updated to match setup.py entry point
            name='joystick_controller',
            parameters=[{
                'use_direct_control': False,  # Flag to indicate controller-based control
                'publish_rate': 100.0,  # Hz
                'max_joint_velocity': 1.0,  # rad/s
                'joint_names': [
                    'Rotation', 'Pitch', 'Elbow', 
                    'Wrist_Pitch', 'Wrist_Roll', 'Jaw'
                ],
            }],
            remappings=[
                # Use controller topics instead of direct joint commands
                ('joint_commands', '/joint_group_position_controller/commands'),
                ('gripper_command', '/jaw_controller/gripper_cmd')
            ]
        )
    ]) 