# Joystick Controller System

## Overview

The joystick controller system provides intuitive control for the SO-ARM 100 robot through both physical joysticks and keyboard input. It consists of two main components that work together to convert user input into robot joint commands.

## Architecture

```
User Input → Input Controller → Joy Messages → Robot Controller → Robot Commands
```

### Components

1. **Input Controllers** (Choose one):
   - **Physical Joystick**: Uses actual joystick hardware
   - **Keyboard Joystick**: Simulates joystick input using keyboard

2. **Robot Controller**: Converts Joy messages into robot joint commands

## Message Flow

### Input Controllers
- **Input**: User input (joystick axes/buttons or keyboard keys)
- **Output**: `sensor_msgs/Joy` messages published to `/joy` topic

### Robot Controller
- **Input**: `sensor_msgs/Joy` messages from `/joy` topic
- **Output**: Robot joint commands to:
  - `/joint_trajectory_controller/follow_joint_trajectory` (trajectory_msgs/JointTrajectory)
  - `/joint_group_position_controller/commands` (std_msgs/Float64MultiArray)

## Control Mappings

### Physical Joystick Mapping
- **Left Stick X** → Base Rotation
- **Left Stick Y** → Shoulder Pitch
- **Right Stick X** → Elbow Pitch
- **Right Stick Y** → Wrist Pitch
- **Triggers** → Wrist Roll

### Keyboard Joystick Mapping
```
Left Stick (Base & Shoulder):
  W/S - Shoulder pitch (up/down)
  A/D - Base rotation (left/right)

Right Stick (Elbow & Wrist):
  I/K - Wrist pitch (up/down)
  J/L - Elbow pitch (left/right)

Triggers (Wrist Roll):
  Q/E - Wrist roll (left/right)

Buttons:
  Space - Button 0 (Gripper toggle)
  R - Button 1 (Reset positions)
  F - Button 2 (Fine control mode)
  Ctrl+C - Exit
```

## Usage Options

### Option 1: Physical Joystick (Requires USB Joystick)
```bash
# Start the joystick controller
./start_robostack.sh
# Choose option 1: "Start joystick controller for robot control"
```

### Option 2: Keyboard Joystick (No Hardware Required)
```bash
# Start the keyboard joystick controller
./start_robostack.sh
# Choose option 2: "Start keyboard joystick controller"
```

### Option 3: Manual Setup (Advanced Users)
```bash
# Terminal 1: Start keyboard joystick
docker compose exec robostack bash -c "cd /ros2_ws && source install/setup.bash && ./install/joystick_controller/bin/keyboard_joystick"

# Terminal 2: Start joystick controller
docker compose exec robostack bash -c "cd /ros2_ws && source install/setup.bash && ./install/joystick_controller/bin/joystick_controller"
```

## Features

### Keyboard Joystick Features
- ✅ **No hardware required** - Works with any keyboard
- ✅ **Fine control mode** - Press 'F' to toggle reduced sensitivity
- ✅ **Real-time input** - 10Hz publishing rate
- ✅ **Gradual decay** - Smooth axis reset when keys are released
- ✅ **Remote friendly** - Works over SSH/remote connections

### Robot Controller Features
- ✅ **Joint limits** - Prevents unsafe joint positions
- ✅ **Smooth trajectories** - Generates proper joint trajectories
- ✅ **Multiple interfaces** - Supports both trajectory and direct commands
- ✅ **Configurable sensitivity** - Adjustable control responsiveness

## Technical Details

### ROS2 Topics

**Published Topics:**
- `/joy` (sensor_msgs/Joy) - Joystick input data
- `/joint_trajectory_controller/follow_joint_trajectory` (trajectory_msgs/JointTrajectory)
- `/joint_group_position_controller/commands` (std_msgs/Float64MultiArray)

**Subscribed Topics:**
- `/joy` (sensor_msgs/Joy) - Joystick input data

### Joint Names (SO-ARM 100)
```python
joint_names = [
    'base_rotation_joint',      # Joint 1 - Base rotation
    'shoulder_pitch_joint',     # Joint 2 - Shoulder pitch
    'elbow_pitch_joint',        # Joint 3 - Elbow pitch
    'wrist_pitch_joint',        # Joint 4 - Wrist pitch
    'wrist_roll_joint'          # Joint 5 - Wrist roll
]
```

### Joint Limits
```python
limits = [
    (-np.pi, np.pi),      # Base rotation: full 360°
    (-np.pi/2, np.pi/2),  # Shoulder pitch: ±90°
    (-np.pi/2, np.pi/2),  # Elbow pitch: ±90°
    (-np.pi/2, np.pi/2),  # Wrist pitch: ±90°
    (-np.pi, np.pi)       # Wrist roll: full 360°
]
```

## Troubleshooting

### Common Issues

1. **"No executable found"**
   - Solution: Rebuild the package
   ```bash
   docker compose exec robostack bash -c "cd /ros2_ws && colcon build --packages-select joystick_controller"
   ```

2. **Keyboard not responding**
   - Solution: Ensure you're in the correct terminal and no other process is capturing input

3. **Robot not moving**
   - Check if Isaac Sim is running and publishing joint states
   - Verify ROS2 topics are active: `ros2 topic list`

4. **Erratic movement**
   - Try fine control mode (press 'F')
   - Check joint limits and sensitivity settings

### Debug Commands
```bash
# Check if topics are publishing
ros2 topic echo /joy

# Check robot joint states
ros2 topic echo /isaac_joint_states

# Check robot commands
ros2 topic echo /isaac_joint_commands

# List all active topics
ros2 topic list
```

## Integration with Isaac Sim

The joystick controller system integrates with Isaac Sim through:

1. **ROS2 Bridge**: Isaac Sim publishes joint states via `/isaac_joint_states`
2. **Joint Commands**: Controller publishes commands via `/isaac_joint_commands`
3. **Real-time Control**: 10Hz control loop for responsive manipulation

## Safety Features

- **Joint limits**: Prevents unsafe joint configurations
- **Smooth trajectories**: Avoids sudden movements
- **Gradual decay**: Smooth axis reset prevents jerky motion
- **Fine control**: Reduced sensitivity for precise manipulation

## Future Enhancements

- [ ] **Gripper control**: Add gripper open/close functionality
- [ ] **Preset positions**: Save and recall common robot poses
- [ ] **Velocity control**: Add velocity-based control mode
- [ ] **Force feedback**: Visual feedback for joint limits
- [ ] **Calibration**: Automatic joystick calibration 