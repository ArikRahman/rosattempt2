# Quick Start Guide - TI mmWave Radar

## Prerequisites

1. **Hardware Connected:**
   - TI mmWave radar board connected via USB
   - DCA1000EVM connected via Ethernet
   - Power supplied to both devices

2. **Network Configured:**
   ```bash
   # Find your ethernet interface
   ip link show
   
   # Configure network (replace eth0 with your interface)
   sudo ip addr add 192.168.33.30/24 dev eth0
   
   # Verify connection to DCA1000EVM
   ping 192.168.33.180
   ```

3. **Serial Port Permissions:**
   ```bash
   # Add yourself to dialout group (one-time setup)
   sudo usermod -a -G dialout $USER
   # Then log out and back in
   
   # OR run with sudo (temporary)
   ```

## Running the Radar

### Option 1: Easiest Way (Using Helper Script)

```bash
cd /home/arik/rosattempt2
./run_radar.sh
```

### Option 2: Step-by-Step

```bash
# Terminal 1: Start ROS master
cd /home/arik/rosattempt2
nix develop
source ws/devel/setup.bash
roscore

# Terminal 2: Run radar node
cd /home/arik/rosattempt2
nix develop
source ws/devel/setup.bash
rosrun mmWave no_Qt.py --cmd_tty /dev/ttyACM0 14xx/indoor_human_rcs

# Terminal 3: Visualize data (optional)
cd /home/arik/rosattempt2
nix develop
source ws/devel/setup.bash
rosrun mmWave fft_viz.py
```

## Checking if it Works

1. **Check ROS topics:**
   ```bash
   # In a new terminal
   cd /home/arik/rosattempt2
   nix develop
   source ws/devel/setup.bash
   rostopic list
   # Should show: /radar_data and /config_string
   ```

2. **Monitor data rate:**
   ```bash
   rostopic hz /radar_data
   # Should show the frame rate
   ```

3. **Echo data:**
   ```bash
   rostopic echo /radar_data
   # Will show raw ADC samples
   ```

## Common Issues

### "Permission denied" on /dev/ttyACM0
```bash
# Quick fix (requires logout/login after)
sudo usermod -a -G dialout $USER

# Immediate fix (one session only)
sudo chmod 666 /dev/ttyACM0
```

### Wrong serial port
```bash
# Find the correct port
ls -l /dev/ttyACM*
# Try /dev/ttyACM1 if ACM0 doesn't work
./run_radar.sh 14xx/indoor_human_rcs /dev/ttyACM1
```

### "Cannot connect to DCA1000EVM"
```bash
# Check network
ip addr show
ping 192.168.33.180

# Reconfigure if needed
sudo ip addr add 192.168.33.30/24 dev <your_interface>
```

### "Package 'mmWave' not found"
```bash
# Source the workspace
cd /home/arik/rosattempt2
source ws/devel/setup.bash
```

## Available Configurations

- `14xx/indoor_human_rcs` - Best for indoor testing (default)
- `14xx/outdoor_human_rcs_30m` - Outdoor, 30m range
- `14xx/outdoor_human_rcs_50m` - Outdoor, 50m range
- `miso` - MISO configuration
- `tdma2` - TDMA with 2 transmitters
- `tdma3` - TDMA with 3 transmitters

## Next Steps

Once the radar is running:
- Check `SETUP_README.md` for detailed documentation
- Modify configurations in `ws/src/mmWave/scripts/configs/`
- Process data using `fft_viz.py` or `listener.py`
- Write your own data processing nodes

## Getting Help

1. Check terminal output for error messages
2. Verify all hardware connections
3. Ensure network and serial permissions are correct
4. See `SETUP_README.md` for troubleshooting details
