#!/bin/bash

echo "=== Isaac Sim Headless WebRTC Streaming ==="
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

# Check for WebRTC client
if [ -f "$HOME/Downloads/isaacsim-webrtc-streaming-client-1.0.6-linux-x64.AppImage" ]; then
    echo "Found Isaac Sim WebRTC streaming client in Downloads"
    echo "This is the recommended way to view the stream!"
    echo ""
    echo "If you encounter GPU/display issues, try:"
    echo "  ./fix_webrtc_client.sh"
    echo ""
else
    echo "Note: Isaac Sim WebRTC streaming client not found in Downloads"
    echo "You can download it from NVIDIA's website for better streaming experience"
fi

echo "Starting Isaac Sim with WebRTC streaming..."
echo "This will run in headless mode but provide a web stream."
echo ""

# Start the WebRTC-enabled headless simulation
docker compose up --remove-orphans isaac-sim-headless-webrtc

echo ""
echo "=== WebRTC Stream Access ==="
echo ""
echo "OPTION 1 (Recommended): Use the Isaac Sim WebRTC Client"
echo "  ./fix_webrtc_client.sh"
echo "  Or run directly: ~/Downloads/isaacsim-webrtc-streaming-client-1.0.6-linux-x64.AppImage"
echo "  Then connect to: localhost:8211"
echo ""
echo "OPTION 2: Web Browser (basic functionality)"
echo "  http://localhost:8211"
echo ""
echo "If running on a remote server, use your server's IP instead of localhost"
echo "To get your server IP: curl ifconfig.me"
echo ""
echo "=== Troubleshooting ==="
echo "1. If the stream doesn't load, wait a few minutes for initialization"
echo "2. Check logs: docker logs isaacsim450-headless-webrtc"
echo "3. Ensure port 8211 is open in your firewall"
echo "4. For remote access, you may need to set up port forwarding"
echo "5. If WebRTC client has GPU issues, try: ./fix_webrtc_client.sh"
echo "6. The AppImage client provides better performance than web browsers"
echo ""
echo "=== Stopping ==="
echo "To stop the simulation: Ctrl+C or 'docker compose down'" 