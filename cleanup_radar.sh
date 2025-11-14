#!/bin/bash
# Cleanup script to force radar system to clean state
# Use this when the radar node gets stuck or ports are in use

echo "🧹 Cleaning up radar processes and network ports..."

# Kill all ROS and radar processes
echo "  Killing ROS processes..."
pkill -9 -f "roscore" 2>/dev/null
pkill -9 -f "rosmaster" 2>/dev/null
pkill -9 -f "no_Qt.py" 2>/dev/null
pkill -9 -f "roslaunch" 2>/dev/null
pkill -9 -f "rosrun" 2>/dev/null

# Wait for processes to die
sleep 1

# Check if ports are still in use
echo "  Checking network ports..."
if lsof -i :4096 >/dev/null 2>&1; then
    echo "  ⚠ Port 4096 (DCA command) still in use, force killing..."
    fuser -k 4096/udp 2>/dev/null
fi

if lsof -i :4098 >/dev/null 2>&1; then
    echo "  ⚠ Port 4098 (DCA data) still in use, force killing..."
    fuser -k 4098/udp 2>/dev/null
fi

if lsof -i :11311 >/dev/null 2>&1; then
    echo "  ⚠ Port 11311 (ROS master) still in use, force killing..."
    fuser -k 11311/tcp 2>/dev/null
fi

# Wait for ports to be released
sleep 1

# Verify cleanup
REMAINING=$(ps aux | grep -E "roscore|rosmaster|no_Qt" | grep -v grep | wc -l)
if [ $REMAINING -eq 0 ]; then
    echo "✓ All processes cleaned up successfully"
else
    echo "⚠ Warning: $REMAINING processes still running"
    ps aux | grep -E "roscore|rosmaster|no_Qt" | grep -v grep
fi

# Check port status
echo ""
echo "Port status:"
if lsof -i :4096 >/dev/null 2>&1; then
    echo "  ✗ Port 4096 still in use"
else
    echo "  ✓ Port 4096 free"
fi

if lsof -i :4098 >/dev/null 2>&1; then
    echo "  ✗ Port 4098 still in use"
else
    echo "  ✓ Port 4098 free"
fi

if lsof -i :11311 >/dev/null 2>&1; then
    echo "  ✗ Port 11311 still in use"
else
    echo "  ✓ Port 11311 free"
fi

echo ""
echo "✓ Cleanup complete. System is ready for a fresh start."
echo ""
echo "Next steps:"
echo "  1. Start roscore: nix develop"
echo "                    source ws/devel/setup.bash"
echo "                    roscore &"
echo ""
echo "  2. Run radar:     ./run_radar.sh"
