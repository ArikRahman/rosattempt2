# TI mmWave Radar ROS Node - Setup Guide

This repository contains a working ROS Noetic node for interfacing with Texas Instruments xWR radar boards (14xx, 16xx, 18xx, 68xx series) using the DCA1000EVM capture card.

## Changes Made to Get It Working

### 1. Python 2 to Python 3 Compatibility
Fixed Python 2/3 compatibility issues in the following files:
- `ws/src/mmWave/scripts/circular_buffer.py`
- `ws/src/mmWave/scripts/mmWave_class_noQt.py`
- `ws/src/mmWave/scripts/no_Qt.py`
- `ws/src/mmWave/scripts/radar_config.py`

Changes included:
- Updated `import Queue` to support both Python 2 and 3 with try/except
- Changed integer division `/` to `//` for Python 3 compatibility

### 2. Nix Development Environment
Updated `flake.nix` to include:
- ROS Noetic with all required packages (rospy, roscpp, std_msgs, message_generation, message_runtime)
- Python 3 with necessary packages (numpy, opencv, pyserial)
- Proper environment variables for ROS and library paths

### 3. Build System
Successfully built the ROS workspace with:
- Custom message type (`data_frame.msg`)
- C library for circular buffer (`libcbuffer.so`)

## Hardware Requirements

1. **TI mmWave Radar Board**: xWR14xx, xWR16xx, xWR18xx, or xWR68xx
2. **DCA1000EVM**: Data capture card for LVDS streaming
3. **Network Connection**: Ethernet connection to DCA1000EVM (default IP: 192.168.33.180)
4. **Serial Connection**: USB serial connection to radar board

## Network Setup

The DCA1000EVM must be configured with:
- IP Address: `192.168.33.180`
- Command Port: `4096`
- Data Port: `4098`

Your computer should have a network interface configured as:
- IP Address: `192.168.33.30`
- Subnet: `192.168.33.0/24`

To configure your network interface:
```bash
sudo ip addr add 192.168.33.30/24 dev <your_ethernet_interface>
# Example: sudo ip addr add 192.168.33.30/24 dev eth0
```

## Installation & Build

The project uses Nix flakes for reproducible development environment:

```bash
# Enter the development environment
cd /home/arik/rosattempt2
nix develop

# Build the workspace (already done)
cd ws
catkin_make

# Source the workspace
source devel/setup.bash
```

## Running the Radar Node

### Method 1: Using the Helper Script

```bash
cd /home/arik/rosattempt2
./run_radar.sh [config_name] [command_tty]
```

Examples:
```bash
# Use default configuration (14xx/indoor_human_rcs) and default TTY (/dev/ttyACM0)
./run_radar.sh

# Use outdoor configuration
./run_radar.sh 14xx/outdoor_human_rcs_30m

# Specify custom TTY device
./run_radar.sh 14xx/indoor_human_rcs /dev/ttyACM1
```

### Method 2: Using ROS Launch File

First, start roscore in one terminal:
```bash
cd /home/arik/rosattempt2
nix develop
source ws/devel/setup.bash
roscore
```

Then in another terminal:
```bash
cd /home/arik/rosattempt2
nix develop
source ws/devel/setup.bash
roslaunch mmWave radar_rd_fft_viz.launch xwr_cmd_tty:=/dev/ttyACM0 xwr_radar_cfg:=14xx/indoor_human_rcs
```

### Method 3: Using rosrun Directly

```bash
cd /home/arik/rosattempt2
nix develop
source ws/devel/setup.bash

# Start roscore in background or separate terminal
roscore &

# Run the node
rosrun mmWave no_Qt.py --cmd_tty /dev/ttyACM0 14xx/indoor_human_rcs
```

## Available Configurations

Located in `ws/src/mmWave/scripts/configs/`:

### For xWR14xx:
- `14xx/indoor_human_rcs.cfg` - Indoor human detection
- `14xx/outdoor_human_rcs_30m.cfg` - Outdoor 30m range
- `14xx/outdoor_human_rcs_50m.cfg` - Outdoor 50m range

### Other configurations:
- `miso.cfg` - MISO configuration
- `tdma2.cfg` - TDMA 2 TX configuration
- `tdma3.cfg` - TDMA 3 TX configuration

Additional configurations for xWR18xx are available in `iwr_raw_rosnode/radar_configs/18xx/`.

## ROS Topics

The node publishes on the following topics:

- `/radar_data` (mmWave/data_frame): Raw ADC data from the radar
- `/config_string` (std_msgs/String): Configuration file contents (latched)

To visualize the data, run the FFT visualization node:
```bash
rosrun mmWave fft_viz.py
```

To listen to radar data:
```bash
rosrun mmWave listener.py
```

## Troubleshooting

### Serial Port Permissions
If you get "Permission denied" on `/dev/ttyACM0`:
```bash
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

Or run with sudo (not recommended):
```bash
sudo -E env PATH=$PATH rosrun mmWave no_Qt.py --cmd_tty /dev/ttyACM0 14xx/indoor_human_rcs
```

### Network Connection Issues
1. Verify DCA1000EVM IP configuration
2. Check network interface configuration:
   ```bash
   ip addr show
   ping 192.168.33.180
   ```
3. Ensure no firewall is blocking UDP ports 4096 and 4098

### Radar Not Responding
1. Check USB serial connection: `ls -l /dev/ttyACM*`
2. Verify correct TTY device (may be `/dev/ttyACM1` instead of `/dev/ttyACM0`)
3. Power cycle the radar board
4. Check that the radar firmware supports LVDS streaming

### Build Errors
If you encounter build errors after making changes:
```bash
cd /home/arik/rosattempt2/ws
nix develop
catkin_make clean
catkin_make
```

## File Structure

```
/home/arik/rosattempt2/
├── flake.nix                    # Nix development environment configuration
├── run_radar.sh                 # Helper script to run the radar node
├── iwr_raw_rosnode/            # Original repository files
│   ├── README.md
│   ├── firmware/               # Radar firmware and LVDS streaming code
│   ├── hardware/               # Hardware designs and mounts
│   └── radar_configs/          # Additional radar configurations
└── ws/                         # ROS workspace
    ├── src/
    │   └── mmWave/             # Main ROS package
    │       ├── CMakeLists.txt
    │       ├── package.xml
    │       ├── msg/
    │       │   └── data_frame.msg
    │       ├── launch/
    │       │   └── radar_rd_fft_viz.launch
    │       └── scripts/
    │           ├── circular_buffer.py    # Ring buffer for radar data
    │           ├── circ_buff.c          # C implementation of buffer
    │           ├── fft_viz.py           # FFT visualization
    │           ├── listener.py          # Simple data listener
    │           ├── mmWave_class_noQt.py # Main radar interface class
    │           ├── no_Qt.py             # ROS node entry point
    │           ├── radar_config.py      # Configuration parser
    │           └── configs/             # Radar configuration files
    ├── build/                  # Build artifacts
    └── devel/                  # Development space
        └── lib/
            └── libcbuffer.so   # Compiled circular buffer library
```

## Development

To make changes to the code:

1. Edit files in `ws/src/mmWave/`
2. Rebuild if needed:
   ```bash
   cd /home/arik/rosattempt2/ws
   nix develop
   catkin_make
   ```
3. Test your changes

## Additional Resources

- [TI mmWave Radar Documentation](https://www.ti.com/tool/MMWAVE-SDK)
- [DCA1000EVM User Guide](https://www.ti.com/tool/DCA1000EVM)
- [ROS Noetic Documentation](http://wiki.ros.org/noetic)

## License

See individual files for license information. The original repository is maintained at https://github.com/moodoki/iwr_raw_rosnode.


According to AI Sonnet 4.5, to run radar is cd /home/arik/rosattempt2
./run_radar.sh

or step by step,

cd /home/arik/rosattempt2
nix develop
source ws/devel/setup.bash
roscore &
rosrun mmWave no_Qt.py --cmd_tty /dev/ttyACM0 14xx/indoor_human_rcs