#!/bin/bash
# Helper script to run the mmWave radar ROS node
# Ensures roscore stays alive and monitors both processes

set -e  # Exit on error

# Enter nix development environment and source ROS workspace
cd /home/arik/rosattempt2

echo "============================================================"
echo "mmWave Radar Launcher"
echo "============================================================"
echo ""

CONFIG="${1:-14xx/indoor_human_rcs}"
CMD_TTY="${2:-/dev/ttyACM0}"

echo "Configuration: $CONFIG"
echo "Command TTY: $CMD_TTY"
echo ""

# Check if another instance is already running
if pgrep -f "rosrun mmWave no_Qt.py" > /dev/null; then
    echo "⚠️  ERROR: Radar node is already running!"
    echo ""
    echo "Run cleanup first:"
    echo "  ./cleanup_radar.sh"
    echo ""
    exit 1
fi

# Function to start roscore if needed
start_roscore() {
    if ! pgrep -f "rosmaster" > /dev/null; then
        echo "[1/2] Starting roscore..."
        nix develop --command bash -c "source ws/devel/setup.bash && roscore" &
        ROSCORE_PID=$!
        
        # Wait for roscore to be ready
        echo "      Waiting for roscore to initialize..."
        for i in {1..30}; do
            if timeout 1 bash -c "exec 3<>/dev/tcp/localhost/11311" 2>/dev/null; then
                echo "      ✓ roscore ready on port 11311 (PID: $ROSCORE_PID)"
                return 0
            fi
            sleep 0.5
        done
        
        echo "      ✗ roscore failed to start within 15 seconds"
        return 1
    else
        ROSCORE_PID=$(pgrep -f "rosmaster")
        echo "[1/2] roscore already running (PID: $ROSCORE_PID)"
        return 0
    fi
}

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "============================================================"
    echo "Shutting down..."
    echo "============================================================"
    
    # Send sensorStop to radar if it's running
    if [[ -n "$RADAR_PID" ]] && kill -0 $RADAR_PID 2>/dev/null; then
        echo "Stopping radar node (PID: $RADAR_PID)..."
        kill -TERM $RADAR_PID 2>/dev/null || true
        wait $RADAR_PID 2>/dev/null || true
    fi
    
    echo "Shutdown complete"
    echo ""
    echo "Note: roscore is still running (PID: $ROSCORE_PID)"
    echo "To stop roscore, run: ./cleanup_radar.sh"
}

# Set up signal handlers
trap cleanup EXIT INT TERM

# Start roscore
if ! start_roscore; then
    echo ""
    echo "Failed to start roscore. Exiting."
    exit 1
fi

echo ""
echo "[2/2] Starting radar node..."
echo "============================================================"
echo ""

# Run the radar node in foreground
nix develop --command bash -c "
  source ws/devel/setup.bash
  exec rosrun mmWave no_Qt.py --cmd_tty $CMD_TTY $CONFIG
" &

RADAR_PID=$!

echo ""
echo "============================================================"
echo "Radar node started (PID: $RADAR_PID)"
echo "============================================================"
echo ""
echo "Monitor ROS topics in another terminal:"
echo "  nix develop"
echo "  source ws/devel/setup.bash"
echo "  rostopic list"
echo "  rostopic hz /radar_data"
echo "  rostopic echo /radar_data"
echo ""
echo "Press Ctrl+C to stop..."
echo "============================================================"
echo ""

# Wait for radar node to exit
wait $RADAR_PID
