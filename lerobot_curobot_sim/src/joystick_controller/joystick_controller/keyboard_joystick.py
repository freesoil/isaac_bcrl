#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import threading
import sys
import time

# For non-blocking single-key reading
import platform

if platform.system() == 'Windows':
    import msvcrt  # type: ignore

    def get_key(timeout: float = 0.1) -> str:
        """Return a single key press within *timeout* seconds (Windows)."""
        start = time.time()
        while time.time() - start < timeout:
            if msvcrt.kbhit():
                ch = msvcrt.getwch()
                return ch
            time.sleep(0.01)
        return ''
else:
    import termios
    import tty
    import select

    def get_key(timeout: float = 0.1) -> str:
        """Return a single key press within *timeout* seconds (POSIX)."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([fd], [], [], timeout)
            if rlist:
                ch = sys.stdin.read(1)
                return ch
            return ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

class KeyboardJoystick(Node):
    def __init__(self):
        super().__init__('keyboard_joystick')
        
        # Publisher for Joy messages
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        
        # Joystick state (6 axes, 11 buttons)
        self.axes = [0.0] * 6
        self.buttons = [0] * 11
        
        # Key mappings for SO-ARM 100 robot
        self.key_mappings = {
            # Left stick (axes 0, 1) - Base rotation and shoulder pitch
            'a': (0, -1.0),  # Left stick X (negative)
            'd': (0, 1.0),   # Left stick X (positive)
            'w': (1, -1.0),  # Left stick Y (negative) - shoulder up
            's': (1, 1.0),   # Left stick Y (positive) - shoulder down
            
            # Right stick (axes 2, 3) - Elbow and wrist pitch
            'j': (2, -1.0),  # Right stick X (negative) - elbow
            'l': (2, 1.0),   # Right stick X (positive) - elbow
            'i': (3, -1.0),  # Right stick Y (negative) - wrist pitch up
            'k': (3, 1.0),   # Right stick Y (positive) - wrist pitch down
            
            # Triggers (axes 4, 5) - Wrist roll
            'q': (4, -1.0),  # Left trigger - wrist roll left
            'e': (5, -1.0),  # Right trigger - wrist roll right
            
            # Buttons for additional controls
            ' ': (0, 1),     # Space - button 0 (gripper toggle)
            'r': (1, 1),     # R - button 1 (reset positions)
            'f': (2, 1),     # F - button 2 (fine control mode)
        }
        
        # Fine control mode (reduced sensitivity)
        self.fine_control = False
        self.sensitivity = 0.3 if self.fine_control else 1.0
        
        # Start keyboard input thread
        self.running = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_input_loop)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        # Timer for publishing joy messages
        self.timer = self.create_timer(0.1, self.publish_joy)  # 10 Hz
        
        self.get_logger().info('Keyboard joystick initialized')
        self.get_logger().info('Controls:')
        self.get_logger().info('  WASD - Left stick (Base rotation, Shoulder pitch)')
        self.get_logger().info('  IJKL - Right stick (Elbow, Wrist pitch)')
        self.get_logger().info('  QE - Triggers (Wrist roll)')
        self.get_logger().info('  Space - Button 0 (Gripper toggle)')
        self.get_logger().info('  R - Button 1 (Reset positions)')
        self.get_logger().info('  F - Button 2 (Fine control mode)')
        self.get_logger().info('  Ctrl+C - Exit')
        
    def keyboard_input_loop(self):
        """Continuously poll for key presses without requiring Enter."""
        try:
            while self.running:
                key = get_key(0.05).lower()

                if not key:
                    continue  # no key press this cycle
                    
                if key == '\x03':  # Ctrl+C
                        self.running = False
                        break
                    
                if key in self.key_mappings:
                        axis_or_button, value = self.key_mappings[key]
                        
                        if isinstance(value, float):  # Axis control
                            self.axes[axis_or_button] = value * self.sensitivity
                        else:  # Button control
                            if key == 'f':  # Toggle fine control
                                self.fine_control = not self.fine_control
                                self.sensitivity = 0.3 if self.fine_control else 1.0
                                self.get_logger().info(
                                    f'Fine control: {"ON" if self.fine_control else "OFF"}'
                                )
                            else:
                                self.buttons[axis_or_button] = value
        except Exception as e:
            self.get_logger().error(f'Keyboard input error: {e}')
    
    def publish_joy(self):
        """Publish Joy message"""
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.axes = self.axes
        joy_msg.buttons = self.buttons
        self.joy_pub.publish(joy_msg)
        
        # Reset axes to zero (for non-held keys)
        for i in range(len(self.axes)):
            if abs(self.axes[i]) > 0.01:  # Small threshold
                self.axes[i] *= 0.8  # Gradual decay
            else:
                self.axes[i] = 0.0

        # After publishing, reset button presses (momentary)
        for i in range(len(self.buttons)):
            self.buttons[i] = 0

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardJoystick()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 