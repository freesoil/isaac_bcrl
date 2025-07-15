# Isaac Manipulator

## Assets
- SO-ARM100 URDF and mesh files: [Follow this link](https://github.com/haosulab/ManiSkill/tree/main/mani_skill/assets/robots/so100/SO_5DOF_ARM100_8j)

## Quick Start

### Building Docker Images

```bash
# Build all Docker images
cd dockers
./isaacsim450/build.sh
./robostack/build.sh
```

### Running the Simulation

This project provides two main simulation modes to suit different use cases:

#### 1. WebRTC Streaming Mode (Recommended for Remote Access)
**Use Case**: Remote viewing and monitoring, headless servers
**Script**: `start_webrtc_stream.sh` (uses `simulation.py --mode webrtc`)

```bash
./start_webrtc_stream.sh
```

**Features**:
- Headless simulation with WebRTC streaming
- Accessible from any device with a web browser
- Real-time robot control via joystick/keyboard
- Camera feeds and sensor data streaming

#### 2. GUI Mode (Local Development)
**Use Case**: Local development, interactive control, machines with display
**Script**: `start_gui.sh` (uses `simulation.py --mode gui`)

```bash
./start_gui.sh
```

**Features**:
- Local GUI controls
- Direct interaction with the simulation
- No streaming client required
- Full Isaac Sim interface

### Robot Control

#### Joystick Controller
Control the SO-ARM 100 robot using physical joysticks or keyboard:

```bash
./start_robostack.sh
# Choose option 1: "Start joystick controller for robot control"
# Choose option 2: "Start keyboard joystick controller"
```

**Keyboard Controls**:
- **WASD**: Left stick (Base rotation, Shoulder pitch)
- **IJKL**: Right stick (Elbow, Wrist pitch)
- **QE**: Triggers (Wrist roll)
- **Space**: Button 0 (Gripper toggle)
- **R**: Button 1 (Reset positions)
- **F**: Button 2 (Fine control mode)

#### Troubleshooting WebRTC
If you have issues with WebRTC streaming:

```bash
./fix_webrtc_client.sh
```

## Native Setup

### Links

- [Isaac Sim 4.5.0 Installation Page](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/download.html)
- [cuRobo Installation Page](https://curobo.org/get_started/1_install_instructions.html)
- [cuRobo Isaac Sim Demos](https://curobo.org/get_started/2b_isaacsim_examples.html)

### Setup

1. **Ensure CUDA 11.8 is installed and configured**

   Add the following lines to your `~/.bashrc` file:

   ```bash
   ## CUDA
   export CUDA_HOME=/usr/local/cuda
   export PATH=/usr/local/cuda/bin:$PATH
   export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
   export LIBRARY_PATH=/usr/local/cuda/lib64:$LIBRARY_PATH
   ```
   To verify that CUDA is installed correctly, run:

   ```bash
   nvcc --version
   ```

2. **Download Isaac Sim** to your home directory.

3. **Add Isaac Sim environment variables and aliases**

   Add the following lines to your `~/.bashrc`:

   ```bash
   export ISAACSIM_PATH="${HOME}/isaacsim"
   alias isaac_sim="${ISAACSIM_PATH}/isaac-sim.sh"
   alias omni_python="${ISAACSIM_PATH}/python.sh"
   ```

4. **Install cuRobo for Isaac Sim**

   Follow the instructions in the official guide for Isaac Sim installation section:
   [cuRobo Install for Isaac Sim](https://curobo.org/get_started/1_install_instructions.html#install-for-use-in-isaac-sim)

## Simulation Modes

The project uses a single, unified simulation script (`lerobot_curobot_sim/simulation.py`) that supports multiple modes:

```bash
# GUI mode (local controls)
python simulation.py --mode gui

# WebRTC streaming mode (remote access)
python simulation.py --mode webrtc --port 8211

# Headless mode (no display)
python simulation.py --mode headless

# With target object
python simulation.py --mode webrtc --port 8211 --show-target
```

## Commands Reference

```bash
# Start headless mode with WebRTC streaming
./start_webrtc_stream.sh

# Start GUI mode with local controls
./start_gui.sh

# Start robot controller (joystick/keyboard)
./start_robostack.sh

# Run simulation directly
python lerobot_curobot_sim/simulation.py --mode webrtc --port 8211

# Check container status
docker compose ps

