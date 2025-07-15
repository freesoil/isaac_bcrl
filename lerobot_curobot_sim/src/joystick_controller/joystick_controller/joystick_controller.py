#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray
from control_msgs.msg import GripperCommand

class JoystickController(Node):
    def __init__(self):
        super().__init__('joystick_controller')

        # Declare parameters
        self.declare_parameter('use_direct_control', True)
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('max_joint_velocity', 1.0)
        self.declare_parameter('joint_names', [
            'Rotation', 'Pitch', 'Elbow', 
            'Wrist_Pitch', 'Wrist_Roll', 'Jaw'
        ])

        # Get parameters
        self.use_direct_control = self.get_parameter('use_direct_control').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.max_joint_velocity = self.get_parameter('max_joint_velocity').value
        self.joint_names = self.get_parameter('joint_names').value

        # Initialize joint positions
        self.joint_positions = {name: 0.0 for name in self.joint_names}
        
        # Fine control mode (slower movements)
        self.fine_control = False
        
        # Subscribe to joy messages
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        # Create publishers based on control mode
        if self.use_direct_control:
            # Direct joint control mode
            self.joint_pub = self.create_publisher(
                JointState,
                'isaac_joint_commands',
                10
            )
        else:
            # Controller-based mode
            self.arm_pub = self.create_publisher(
                Float64MultiArray,
                'joint_commands',  # Will be remapped to /joint_group_position_controller/commands
                10
            )
            self.gripper_pub = self.create_publisher(
                GripperCommand,
                'gripper_command',  # Will be remapped to /jaw_controller/gripper_cmd
                10
            )

        # Create timer for publishing commands
        self.timer = self.create_timer(1.0/self.publish_rate, self.timer_callback)

        self.get_logger().info(
            f"Joystick controller started in {'direct' if self.use_direct_control else 'controller-based'} mode"
        )

    def joy_callback(self, msg):
        # Get control mode buttons
        if msg.buttons[0]:  # R button
            self.fine_control = True
        elif msg.buttons[1]:  # F button
            self.fine_control = False

        # Calculate velocity scaling
        scale = 0.2 if self.fine_control else 1.0
        dt = 1.0/self.publish_rate
        delta = self.max_joint_velocity * scale * dt

        # Update joint positions based on joystick axes
        # Left stick X: Base rotation
        self.joint_positions['Rotation'] += msg.axes[0] * delta
        
        # Left stick Y: Shoulder pitch
        self.joint_positions['Pitch'] += msg.axes[1] * delta
        
        # Right stick Y: Elbow
        self.joint_positions['Elbow'] += msg.axes[4] * delta
        
        # Right stick X: Wrist pitch
        self.joint_positions['Wrist_Pitch'] += msg.axes[3] * delta
        
        # D-pad X: Wrist roll
        if len(msg.axes) > 6:  # Some joysticks have D-pad as axes
            self.joint_positions['Wrist_Roll'] += msg.axes[6] * delta
        
        # Gripper control
        if msg.buttons[4]:  # Open gripper
            self.joint_positions['Jaw'] += delta
        elif msg.buttons[5]:  # Close gripper
            self.joint_positions['Jaw'] -= delta

        # Apply joint limits
        self.joint_positions['Rotation'] = max(-2.1, min(2.1, self.joint_positions['Rotation']))
        self.joint_positions['Pitch'] = max(-0.1, min(3.45, self.joint_positions['Pitch']))
        self.joint_positions['Elbow'] = max(-0.2, min(3.14159, self.joint_positions['Elbow']))
        self.joint_positions['Wrist_Pitch'] = max(-1.8, min(1.8, self.joint_positions['Wrist_Pitch']))
        self.joint_positions['Wrist_Roll'] = max(-3.14159, min(3.14159, self.joint_positions['Wrist_Roll']))
        self.joint_positions['Jaw'] = max(0.0, min(0.8, self.joint_positions['Jaw']))  # Assuming 0.8 rad max opening

    def timer_callback(self):
        if self.use_direct_control:
            # Publish all joint positions directly
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = [self.joint_positions[name] for name in self.joint_names]
            self.joint_pub.publish(msg)
        else:
            # Publish arm positions to controller
            arm_msg = Float64MultiArray()
            arm_msg.data = [self.joint_positions[name] for name in self.joint_names[:-1]]  # All except Jaw
            self.arm_pub.publish(arm_msg)

            # Publish gripper command to controller
            gripper_msg = GripperCommand()
            gripper_msg.position = self.joint_positions['Jaw']
            gripper_msg.max_effort = -1.0  # Use default effort
            self.gripper_pub.publish(gripper_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 