#!/bin/bash

echo "=== Isaac Sim GUI Controls ==="
echo ""

# Check if we're in the right directory
if [ ! -f "docker-compose.yml" ]; then
    echo "Error: Please run this script from the isaac_manipulator directory"
    exit 1
fi

# Create cache directories
echo "Creating cache directories..."
mkdir -p ~/docker/isaac-sim/cache/{kit,ov,pip,glcache,computecache,logs,data,documents}

# Check if the simulation script exists
if [ ! -f "lerobot_curobot_sim/simulation.py" ]; then
    echo "Error: simulation.py not found"
    exit 1
fi

# Set up X11 forwarding for Docker
echo "Setting up X11 forwarding for Docker..."
if [ -z "$DISPLAY" ]; then
    echo "Warning: No DISPLAY environment variable set"
    echo "Setting DISPLAY to :0 (default)"
    export DISPLAY=:0
fi

# Allow X11 connections from Docker containers
echo "Allowing X11 connections from Docker containers..."
xhost +local:docker 2>/dev/null || xhost +local: 2>/dev/null || echo "Warning: Could not set xhost permissions"

echo "DISPLAY is set to: $DISPLAY"
echo ""

echo "Starting Isaac Sim with GUI controls..."
echo "This will provide local GUI controls without streaming."
echo ""

# Start the GUI simulation using the unified script
docker compose up --remove-orphans isaac-sim-gui

echo ""
echo "=== GUI Controls ==="
echo ""
echo "The Isaac Sim window should open with full controls:"
echo "  - Mouse: Look around"
echo "  - WASD: Move camera"
echo "  - Q/E: Roll camera"
echo "  - R/F: Zoom in/out"
echo "  - Space: Pause/Resume simulation"
echo ""
echo "=== Troubleshooting ==="
echo "1. If GUI doesn't open, try: xhost +local:"
echo "2. Check logs: docker logs isaacsim450-gui"
echo "3. Ensure your remote desktop session is active"
echo "4. Try restarting your remote desktop session"
echo ""
echo "=== Stopping ==="
echo "To stop the simulation: Ctrl+C or 'docker compose down'" 