from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Keyboard to Joy converter node
        Node(
            package='joystick_controller',
            executable='keyboard_joystick',
            name='keyboard_joystick',
            parameters=[{
                'publish_rate': 100.0,  # Hz
                'key_timeout': 0.1,  # seconds
                'key_mappings': {
                    'w': {'axis': 1, 'value': 1.0},   # Forward
                    's': {'axis': 1, 'value': -1.0},  # Backward
                    'a': {'axis': 0, 'value': -1.0},  # Left
                    'd': {'axis': 0, 'value': 1.0},   # Right
                    'i': {'axis': 4, 'value': 1.0},   # Wrist up
                    'k': {'axis': 4, 'value': -1.0},  # Wrist down
                    'j': {'axis': 3, 'value': -1.0},  # Wrist left
                    'l': {'axis': 3, 'value': 1.0},   # Wrist right
                    'q': {'button': 4, 'value': 1},   # Open gripper
                    'e': {'button': 5, 'value': 1},   # Close gripper
                    'r': {'button': 0, 'value': 1},   # Fine control mode
                    'f': {'button': 1, 'value': 1},   # Normal control mode
                }
            }],
            output='screen'
        )
    ]) 
