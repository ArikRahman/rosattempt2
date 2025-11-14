#!/bin/bash
# Helper script to run the mmWave radar ROS node

# Enter nix development environment and source ROS workspace
cd /home/arik/rosattempt2

echo "Starting mmWave radar node..."
echo ""
echo "Usage: $0 [config_name] [command_tty]"
echo "  config_name: Configuration file name without .cfg extension (default: 14xx/indoor_human_rcs)"
echo "  command_tty: TTY device for radar commands (default: /dev/ttyACM0)"
echo ""

CONFIG="${1:-14xx/indoor_human_rcs}"
CMD_TTY="${2:-/dev/ttyACM0}"

echo "Using configuration: $CONFIG"
echo "Using command TTY: $CMD_TTY"
echo ""

# Check if roscore is running by attempting to connect to ROS master
echo "Checking for roscore..."
if ! timeout 2 bash -c "exec 3<>/dev/tcp/localhost/11311" 2>/dev/null; then
    echo "⚠️  WARNING: roscore is not running!"
    echo ""
    echo "Please start roscore in another terminal:"
    echo "  cd /home/arik/rosattempt2"
    echo "  nix develop"
    echo "  source ws/devel/setup.bash"
    echo "  roscore"
    echo ""
    echo "Then run this script again."
    echo ""
    exit 1
fi

echo "✓ roscore is running on port 11311"
echo ""

# Source the workspace and run the node
nix develop --command bash -c "
  source ws/devel/setup.bash
  rosrun mmWave no_Qt.py --cmd_tty $CMD_TTY $CONFIG
"
