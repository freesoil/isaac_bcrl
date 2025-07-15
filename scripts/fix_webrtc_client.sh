#!/bin/bash

echo "=== Fixing Isaac Sim WebRTC Client Issues ==="
echo ""

# Check if the AppImage exists
CLIENT_PATH="$HOME/Downloads/isaacsim-webrtc-streaming-client-1.0.6-linux-x64.AppImage"

if [ ! -f "$CLIENT_PATH" ]; then
    echo "Error: Isaac Sim WebRTC streaming client not found at:"
    echo "  $CLIENT_PATH"
    exit 1
fi

echo "Found WebRTC client at: $CLIENT_PATH"
echo ""

# Make it executable
echo "Making the client executable..."
chmod +x "$CLIENT_PATH"

# Check if we're in a headless environment
if [ -z "$DISPLAY" ]; then
    echo "Warning: No DISPLAY environment variable set"
    echo "This might cause issues with the WebRTC client"
fi

echo ""
echo "=== Running WebRTC Client with GPU Fixes ==="
echo ""

# Run the client with various GPU/display fixes
echo "Attempting to run with GPU fixes..."

# Method 1: Try with software rendering fallback
echo "Method 1: Software rendering fallback..."
"$CLIENT_PATH" --enable-unsafe-swiftshader --disable-gpu-sandbox --disable-software-rasterizer 2>/dev/null &
CLIENT_PID=$!

# Wait a moment to see if it starts
sleep 3

if kill -0 $CLIENT_PID 2>/dev/null; then
    echo "Client started successfully with Method 1"
    echo "Connect to: localhost:8211"
    echo "Press Ctrl+C to stop"
    wait $CLIENT_PID
else
    echo "Method 1 failed, trying Method 2..."
    
    # Method 2: Try with different GPU settings
    echo "Method 2: Alternative GPU settings..."
    DISPLAY=:0 "$CLIENT_PATH" --disable-gpu --disable-gpu-sandbox --disable-software-rasterizer 2>/dev/null &
    CLIENT_PID=$!
    
    sleep 3
    
    if kill -0 $CLIENT_PID 2>/dev/null; then
        echo "Client started successfully with Method 2"
        echo "Connect to: localhost:8211"
        echo "Press Ctrl+C to stop"
        wait $CLIENT_PID
    else
        echo "Method 2 failed, trying Method 3..."
        
        # Method 3: Try with minimal settings
        echo "Method 3: Minimal settings..."
        "$CLIENT_PATH" --no-sandbox --disable-gpu --disable-gpu-sandbox --disable-dev-shm-usage 2>/dev/null &
        CLIENT_PID=$!
        
        sleep 3
        
        if kill -0 $CLIENT_PID 2>/dev/null; then
            echo "Client started successfully with Method 3"
            echo "Connect to: localhost:8211"
            echo "Press Ctrl+C to stop"
            wait $CLIENT_PID
        else
            echo "All methods failed. Trying to run directly..."
            echo "This might show error messages to help diagnose the issue."
            "$CLIENT_PATH"
        fi
    fi
fi

echo ""
echo "=== Troubleshooting Tips ==="
echo "1. If the client won't start, try running it directly:"
echo "   $CLIENT_PATH"
echo ""
echo "2. Check if you have X11 forwarding enabled:"
echo "   echo \$DISPLAY"
echo ""
echo "3. For remote access, you might need to set up X11 forwarding:"
echo "   ssh -X user@server"
echo ""
echo "4. Alternative: Use web browser access at http://localhost:8211"