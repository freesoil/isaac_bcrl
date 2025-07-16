#!/bin/bash

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== Isaac Sim Robot Controller ===${NC}"
echo ""
echo "What would you like to do?"
echo "1) Start direct joint control with joystick"
echo "2) Start controller-based joint control with joystick"
echo "3) Start keyboard control (can be used with either control mode)"
echo "4) Run MPC demo"
echo "5) Run motion generation demo"
echo "6) Open interactive shell in robostack container"
echo "7) Open interactive shell in Isaac Sim container"
echo "8) Show joystick controller help/documentation"
echo "9) Exit"
echo ""

read -p "Enter your choice (1-9): " choice

check_and_start_robostack() {
    # Check if robostack container is running
    if ! docker compose ps robostack | grep -q "Up"; then
        echo -e "${YELLOW}Starting robostack container...${NC}"
        docker compose up -d robostack
        sleep 3
    fi
}

# --- NEW: ensure executables exist where ROS2 expects them --------------------
ensure_libexec_symlinks() {
    # Inside the container, ROS 2 expects executables to live in
    # /ros2_ws/install/joystick_controller/lib/joystick_controller/
    # Some environments (e.g. pip install) place them in the bin/ directory
    # instead.  If that happens, launches will fail with:
    #   "libexec directory '/ros2_ws/install/joystick_controller/lib/joystick_controller' does not exist"
    # This helper checks for that case and creates symlinks so launches work
    # regardless of where the scripts were installed.
    docker compose exec robostack bash -c "\
        if [ -d /ros2_ws/install/joystick_controller ] && \
           [ ! -d /ros2_ws/install/joystick_controller/lib/joystick_controller ]; then \
           echo '# Creating missing libexec directory for joystick_controller'; \
           mkdir -p /ros2_ws/install/joystick_controller/lib/joystick_controller; \
           for f in /ros2_ws/install/joystick_controller/bin/*; do \
               ln -sf \$f /ros2_ws/install/joystick_controller/lib/joystick_controller/\$(basename \$f); \
           done; \
        fi"
}

build_joystick_controller() {
    echo -e "${YELLOW}Building (or rebuilding) joystick controller package...${NC}"
    docker compose exec robostack bash -c "cd /ros2_ws && colcon build --packages-select joystick_controller --symlink-install"
        if [ $? -ne 0 ]; then
            echo -e "${RED}Failed to build joystick controller${NC}"
            exit 1
        fi
    # Ensure executables are where ROS2 expects them
    ensure_libexec_symlinks
}

case $choice in
    1)
        echo -e "${YELLOW}Starting direct joint control with joystick...${NC}"
        check_and_start_robostack
        build_joystick_controller
        
        echo -e "${GREEN}Starting direct joint control...${NC}"
        echo -e "${BLUE}Control mapping:${NC}"
        echo "  Left Stick X -> Base Rotation"
        echo "  Left Stick Y -> Shoulder Pitch"
        echo "  Right Stick Y -> Elbow"
        echo "  Right Stick X -> Wrist Pitch"
        echo "  D-pad X -> Wrist Roll"
        echo "  Buttons 4/5 -> Open/Close Gripper"
        echo "  R Button -> Fine Control Mode (20% speed)"
        echo "  F Button -> Normal Control Mode"
        echo ""
        echo -e "${YELLOW}Press Ctrl+C to stop${NC}"
        echo ""
        
        # Run the direct joint control mode
        docker compose exec -it robostack bash -c "cd /ros2_ws && source install/setup.bash && ros2 launch joystick_controller direct_joy_control.launch.py"
        ;;
    2)
        echo -e "${YELLOW}Starting controller-based joint control with joystick...${NC}"
        check_and_start_robostack
        build_joystick_controller
        
        echo -e "${GREEN}Starting controller-based joint control...${NC}"
        echo -e "${BLUE}Control mapping:${NC}"
        echo "  Left Stick X -> Base Rotation"
        echo "  Left Stick Y -> Shoulder Pitch"
        echo "  Right Stick Y -> Elbow"
        echo "  Right Stick X -> Wrist Pitch"
        echo "  D-pad X -> Wrist Roll"
        echo "  Buttons 4/5 -> Open/Close Gripper"
        echo "  R Button -> Fine Control Mode (20% speed)"
        echo "  F Button -> Normal Control Mode"
        echo ""
        echo -e "${YELLOW}Press Ctrl+C to stop${NC}"
        echo ""
        
        # Run the controller-based joint control mode
        docker compose exec -it robostack bash -c "cd /ros2_ws && source install/setup.bash && ros2 launch joystick_controller controller_joy_control.launch.py"
        ;;
    3)
        echo -e "${YELLOW}Starting keyboard control...${NC}"
        check_and_start_robostack
        build_joystick_controller
        
        echo -e "${GREEN}Starting keyboard control...${NC}"
        echo -e "${BLUE}Keyboard mapping:${NC}"
        echo "  WASD - Left stick (Base rotation, Shoulder pitch)"
        echo "  IJKL - Right stick (Elbow, Wrist pitch)"
        echo "  QE - Wrist roll"
        echo "  Space - Open gripper"
        echo "  B - Close gripper"
        echo "  R - Fine control mode"
        echo "  F - Normal control mode"
        echo ""
        echo -e "${YELLOW}Press Ctrl+C to stop${NC}"
        echo ""
        
        # Run the keyboard control in interactive mode so STDIN attaches to the node
        # ros2 launch detaches STDIN; ros2 run keeps it, allowing key capture
        docker compose exec -it robostack bash -c "cd /ros2_ws && source install/setup.bash && ros2 run joystick_controller keyboard_joystick "
        ;;
    4)
        echo -e "${YELLOW}Running MPC demo...${NC}"
        docker compose run --rm mpc-demo
        ;;
    5)
        echo -e "${YELLOW}Running motion generation demo...${NC}"
        docker compose run --rm motion-gen-demo
        ;;
    6)
        echo -e "${YELLOW}Opening interactive shell in robostack container...${NC}"
        check_and_start_robostack
        
        echo -e "${GREEN}Opening bash shell in robostack container...${NC}"
        echo -e "${BLUE}You can now run ROS2 commands directly in the container.${NC}"
        echo -e "${BLUE}Common commands:${NC}"
        echo "  ros2 topic list"
        echo "  ros2 node list"
        echo "  ros2 topic echo /joy"
        echo "  ros2 topic echo /isaac_joint_states"
        echo "  ros2 launch joystick_controller direct_joy_control.launch.py"
        echo "  ros2 launch joystick_controller controller_joy_control.launch.py"
        echo "  ros2 launch joystick_controller keyboard_joy.launch.py"
        echo ""
        echo -e "${YELLOW}Type 'exit' to leave the container shell${NC}"
        echo ""
        
        docker compose exec robostack bash
        ;;
    7)
        echo -e "${YELLOW}Opening interactive shell in Isaac Sim container...${NC}"
        
        # Check if Isaac Sim container is running
        if ! docker compose ps isaac-sim-headless-webrtc | grep -q "Up"; then
            echo -e "${YELLOW}Isaac Sim container is not running.${NC}"
            echo -e "${BLUE}Available Isaac Sim containers:${NC}"
            docker compose ps | grep isaac
            echo ""
            echo -e "${YELLOW}Please start an Isaac Sim container first using:${NC}"
            echo "  ./start_webrtc_stream.sh"
            echo "  or"
            echo "  docker compose up -d isaac-sim-headless-webrtc"
            exit 1
        fi
        
        echo -e "${GREEN}Opening bash shell in Isaac Sim container...${NC}"
        echo -e "${BLUE}You can now run Isaac Sim commands directly in the container.${NC}"
        echo -e "${BLUE}Common commands:${NC}"
        echo "  cd /sim"
        echo "  python simulation.py --mode webrtc"
        echo "  python simulation.py --mode gui"
        echo "  python simulation.py --mode headless"
        echo ""
        echo -e "${BLUE}ROS2 bridge status:${NC}"
        echo "  Check if ROS2 bridge is working:"
        echo "  ls -la /isaac-sim/exts/isaacsim.ros2.bridge/"
        echo ""
        echo -e "${YELLOW}Type 'exit' to leave the container shell${NC}"
        echo ""
        
        docker compose exec isaac-sim-headless-webrtc bash
        ;;
    8)
        echo -e "${BLUE}=== Joystick Controller Documentation ===${NC}"
        echo ""
        if [ -f "docs/joystick_controller.md" ]; then
            cat docs/joystick_controller.md
        else
            echo -e "${RED}Documentation file not found: docs/joystick_controller.md${NC}"
            echo "Please ensure the documentation file exists."
        fi
        echo ""
        echo -e "${YELLOW}Press Enter to continue...${NC}"
        read
        ;;
    9)
        echo -e "${GREEN}Exiting...${NC}"
        exit 0
        ;;
    *)
        echo -e "${RED}Invalid choice. Please run the script again.${NC}"
        exit 1
        ;;
esac 