#!/bin/bash
# Cleanup script to force radar system to clean state
# Use this when the radar node gets stuck or ports are in use

echo "============================================================"
echo "🧹 Radar System Cleanup"
echo "============================================================"
echo ""

# Kill all ROS and radar processes
echo "[1/3] Stopping processes..."
KILLED=0

if pgrep -f "roscore" > /dev/null; then
    echo "      Killing roscore..."
    pkill -9 -f "roscore" 2>/dev/null && ((KILLED++))
fi

if pgrep -f "rosmaster" > /dev/null; then
    echo "      Killing rosmaster..."
    pkill -9 -f "rosmaster" 2>/dev/null && ((KILLED++))
fi

if pgrep -f "no_Qt.py" > /dev/null; then
    echo "      Killing radar node..."
    pkill -9 -f "no_Qt.py" 2>/dev/null && ((KILLED++))
fi

if pgrep -f "roslaunch" > /dev/null; then
    echo "      Killing roslaunch..."
    pkill -9 -f "roslaunch" 2>/dev/null && ((KILLED++))
fi

if pgrep -f "rosrun" > /dev/null; then
    echo "      Killing rosrun..."
    pkill -9 -f "rosrun" 2>/dev/null && ((KILLED++))
fi

if [ $KILLED -eq 0 ]; then
    echo "      ✓ No processes to kill"
else
    echo "      ✓ Killed $KILLED process(es)"
fi

# Wait for processes to die
sleep 1

# Free up ports
echo ""
echo "[2/3] Freeing network ports..."
PORTS_FREED=0

for PORT_SPEC in "4096/udp:DCA_command" "4098/udp:DCA_data" "11311/tcp:ROS_master"; do
    PORT=$(echo $PORT_SPEC | cut -d: -f1)
    NAME=$(echo $PORT_SPEC | cut -d: -f2)
    
    if sudo lsof -i :${PORT%/*} >/dev/null 2>&1; then
        echo "      Freeing port ${PORT%/*} ($NAME)..."
        sudo fuser -k $PORT 2>/dev/null && ((PORTS_FREED++))
    fi
done

if [ $PORTS_FREED -eq 0 ]; then
    echo "      ✓ No ports to free"
else
    echo "      ✓ Freed $PORTS_FREED port(s)"
fi

# Wait for ports to be released
sleep 1

# Verify cleanup
echo ""
echo "[3/3] Verifying cleanup..."

REMAINING=$(ps aux | grep -E "roscore|rosmaster|no_Qt" | grep -v grep | wc -l)
if [ $REMAINING -eq 0 ]; then
    echo "      ✓ All processes stopped"
else
    echo "      ⚠ Warning: $REMAINING process(es) still running"
    ps aux | grep -E "roscore|rosmaster|no_Qt" | grep -v grep | sed 's/^/        /'
fi

echo ""
echo "Port status:"
PORTS_OK=0
PORTS_FAIL=0

for PORT_SPEC in "4096:DCA_command" "4098:DCA_data" "11311:ROS_master"; do
    PORT=$(echo $PORT_SPEC | cut -d: -f1)
    NAME=$(echo $PORT_SPEC | cut -d: -f2)
    
    if sudo lsof -i :$PORT >/dev/null 2>&1; then
        echo "  ✗ Port $PORT ($NAME) still in use"
        ((PORTS_FAIL++))
    else
        echo "  ✓ Port $PORT ($NAME) free"
        ((PORTS_OK++))
    fi
done

echo ""
echo "============================================================"
if [ $REMAINING -eq 0 ] && [ $PORTS_FAIL -eq 0 ]; then
    echo "✓ Cleanup complete! System ready for fresh start."
else
    echo "⚠ Cleanup complete with warnings (see above)"
fi
echo "============================================================"
echo ""
echo "Next step:"
echo "  ./run_radar.sh"
echo ""
