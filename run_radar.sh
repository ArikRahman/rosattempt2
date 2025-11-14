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

# Source the workspace and run the node
nix develop --command bash -c "
  source ws/devel/setup.bash
  rosrun mmWave no_Qt.py --cmd_tty $CMD_TTY $CONFIG
"
