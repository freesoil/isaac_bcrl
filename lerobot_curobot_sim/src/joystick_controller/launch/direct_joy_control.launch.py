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
        
        # Joystick controller node that publishes directly to isaac_joint_commands
        Node(
            package='joystick_controller',
            executable='joystick_controller_node',  # Updated to match setup.py entry point
            name='joystick_controller',
            parameters=[{
                'use_direct_control': True,  # Flag to indicate direct joint control
                'publish_rate': 100.0,  # Hz
                'max_joint_velocity': 3.0,  # rad/s
                'joint_names': [
                    'Rotation', 'Pitch', 'Elbow', 
                    'Wrist_Pitch', 'Wrist_Roll', 'Jaw'
                ],
            }],
            remappings=[
                # No remapping needed as we publish directly to isaac_joint_commands
            ]
        )
    ]) 