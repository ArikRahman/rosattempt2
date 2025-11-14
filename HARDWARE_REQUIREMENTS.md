# Hardware Requirements - TI mmWave Radar System

This document describes the hardware requirements and setup for the iwr_raw_rosnode radar system.

## Required Hardware

### 1. TI mmWave Radar Board
**Required**: One of the following TI Evaluation Modules (EVMs):
- **xWR14xx** - 76-81 GHz radar (recommended for this setup)
- **xWR16xx** - 76-81 GHz radar
- **xWR18xx** - 76-81 GHz radar
- **xWR68xx** - 60-64 GHz radar

**Current Status**: ✓ Connected
- Detected: Texas Instruments AR-DevPack-EVM-012
- XDS110 Debug Probe: Version 02.03.00.11
- Serial Number: R1031041

### 2. DCA1000EVM Data Capture Card
**Required**: TI DCA1000EVM Real-Time Data-Capture Adapter

The DCA1000EVM is essential for capturing the high-speed LVDS data stream from the radar. It connects between the radar board and your computer via:
- **Radar Connection**: 60-pin high-speed connector
- **Computer Connection**: Gigabit Ethernet (RJ45)

**Specifications**:
- Data Rate: Up to 3.2 Gbps (4 lanes × 800 Mbps)
- Interface: UDP over Ethernet
- Power: 5V via barrel jack or USB

**Current Status**: ✗ Not Detected
- Expected IP: 192.168.33.180
- Action needed: See troubleshooting section below

### 3. Computer/Network Interface
**Required**: Gigabit Ethernet Port

**Current Configuration**: ✓ Configured
- Interface: eth0
- IP Address: 192.168.33.30/24
- Link Status: UP
- Connection: Direct ethernet cable to DCA1000EVM

### 4. USB Connection
**Required**: USB 2.0 or higher port

**Current Status**: ✓ Connected
- Port 1: /dev/ttyACM0 (Command/Configuration port)
- Port 2: /dev/ttyACM1 (Optional auxiliary/data port)
- Permissions: ✓ User in dialout group

### 5. Power Supply
**Required**:
- **Radar Board**: 5V DC via USB or external power adapter
- **DCA1000EVM**: 5V DC via barrel jack (2.1mm center-positive)

Typical power consumption: ~2-3W (radar) + ~1W (DCA1000EVM)

## Connection Diagram

```
┌─────────────────┐
│   Computer      │
│                 │
│  ┌──────────┐   │
│  │   eth0   │───┼──── Ethernet Cable ────┐
│  └──────────┘   │                        │
│                 │                        ▼
│  ┌──────────┐   │                 ┌──────────────┐
│  │USB Ports │───┼──── USB Cable───│ DCA1000EVM   │
│  └──────────┘   │                 │              │
└─────────────────┘                 │ IP: .33.180  │
                                    └──────┬───────┘
                                           │
                                    60-pin Connector
                                           │
                                    ┌──────▼───────┐
                                    │  TI mmWave   │
                                    │ Radar Board  │
                                    │ (xWR14xx etc)│
                                    └──────────────┘
```

## Network Configuration Details

### Computer Network Settings
```
Interface: eth0
IP Address: 192.168.33.30
Subnet Mask: 255.255.255.0 (/24)
Gateway: Not required for direct connection
```

### DCA1000EVM Network Settings
The DCA1000EVM must be configured with:
```
IP Address: 192.168.33.180
Subnet Mask: 255.255.255.0
Gateway: 192.168.33.1 (optional)
```

**Ports Used**:
- UDP 4096 - Command port (computer → DCA1000EVM)
- UDP 4098 - Data port (DCA1000EVM → computer)

## Current Hardware Status

### ✓ Working Components
1. **TI Radar Board**: Connected via USB
   - Serial ports: /dev/ttyACM0, /dev/ttyACM1
   - Ready for radar configuration commands

2. **Network Interface**: Properly configured
   - IP 192.168.33.30/24 on eth0
   - Link state: UP
   - Ready for DCA1000EVM communication

3. **Serial Permissions**: Configured
   - User 'arik' in dialout group
   - No sudo required for serial access

### ✗ Needs Attention
1. **DCA1000EVM**: Not responding on network
   - Cannot ping 192.168.33.180
   - See troubleshooting section below

## Troubleshooting

### DCA1000EVM Not Detected

**Symptoms**: Cannot ping 192.168.33.180

**Checklist**:
1. ✓ Power
   - [ ] DCA1000EVM power LED is on
   - [ ] Using 5V DC power supply (not USB power)
   - [ ] Power supply provides at least 1A

2. ✓ Physical Connections
   - [ ] Ethernet cable connected to computer's eth0
   - [ ] Ethernet cable connected to DCA1000EVM
   - [ ] Cable is CAT5e or better (Gigabit capable)
   - [ ] Ethernet link LEDs are blinking on both ends

3. ✓ Network Configuration
   - [ ] Computer IP is 192.168.33.30/24
     ```bash
     ip addr show eth0 | grep 192.168.33.30
     ```
   - [ ] No firewall blocking UDP ports 4096 and 4098
     ```bash
     sudo iptables -L | grep 4096
     ```

4. ✓ DCA1000EVM Configuration
   - [ ] DCA1000EVM has been configured with IP 192.168.33.180
   - [ ] Configuration saved to EEPROM (survives power cycle)
   - [ ] DCA1000EVM rebooted after configuration

**Testing Connection**:
```bash
# Test ping
ping -c 4 192.168.33.180

# Check ARP table (device may not respond to ping)
ip neighbor show 192.168.33.180

# Check if port is reachable (requires nmap)
sudo nmap -sU -p 4096 192.168.33.180
```

### Serial Port Access Issues

**Symptoms**: Permission denied on /dev/ttyACM0

**Solutions**:
```bash
# Check current permissions
ls -l /dev/ttyACM0

# Add user to dialout group (one-time, requires re-login)
sudo usermod -a -G dialout $USER

# Temporary fix (until reboot)
sudo chmod 666 /dev/ttyACM0
```

### Wrong Serial Port

**Symptoms**: Radar not responding to commands

**Solution**: Try the other ACM port
```bash
# List available ports
ls -l /dev/ttyACM*

# The radar typically uses ACM0 for commands
# Try ACM1 if ACM0 doesn't work
./run_radar.sh 14xx/indoor_human_rcs /dev/ttyACM1
```

### USB Device Not Recognized

**Symptoms**: No /dev/ttyACM* devices

**Solutions**:
```bash
# Check if USB device is detected
lsusb | grep -i "texas\|ti"

# Check kernel messages
dmesg | tail -20 | grep -i tty

# Reconnect USB cable and check again
```

## Configuring DCA1000EVM

If your DCA1000EVM needs to be configured with the correct IP address:

### Using TI's DCA1000EVM CLI Tool
1. Download from TI website: [DCA1000EVM Software](https://www.ti.com/tool/DCA1000EVM)
2. Configure network settings via the GUI or CLI
3. Save configuration to EEPROM

### Using mmWave Studio (Windows)
1. Install mmWave Studio from TI
2. Connect to DCA1000EVM
3. Configure IP settings in the network tab
4. Save to EEPROM

### Manual Configuration (Advanced)
The DCA1000EVM can be configured via UDP commands, but this requires the device to be accessible first.

## Verification Commands

Run these commands to verify hardware setup:

```bash
# Check USB radar connection
lsusb | grep -i texas

# Check serial ports
ls -l /dev/ttyACM*

# Check network configuration
ip addr show eth0 | grep 192.168.33

# Test DCA1000EVM connectivity
ping -c 2 192.168.33.180

# Check user permissions
groups | grep dialout

# Full hardware check
cd /home/arik/rosattempt2
nix develop --command bash -c "
  echo '=== Hardware Check ==='
  echo 'USB Devices:' && lsusb | grep -i texas
  echo 'Serial Ports:' && ls -l /dev/ttyACM* 2>/dev/null
  echo 'Network:' && ip addr show eth0 | grep 'inet '
  echo 'DCA1000EVM:' && ping -c 1 -W 1 192.168.33.180 >/dev/null 2>&1 && echo '✓ Reachable' || echo '✗ Not reachable'
  echo 'Permissions:' && groups | grep -q dialout && echo '✓ In dialout group' || echo '✗ Not in dialout'
"
```

## Hardware Purchase Links

- **xWR14xx EVM**: [TI Store - AWR1443BOOST](https://www.ti.com/tool/AWR1443BOOST)
- **xWR18xx EVM**: [TI Store - IWR1843BOOST](https://www.ti.com/tool/IWR1843BOOST)
- **DCA1000EVM**: [TI Store - DCA1000EVM](https://www.ti.com/tool/DCA1000EVM)

## Additional Information

### Compatible Radar Boards
This software has been tested with:
- AWR1443BOOST
- IWR1843BOOST
- IWR1642BOOST
- IWR6843AOPEVM

### System Requirements
- **OS**: Linux (tested on Ubuntu/Debian)
- **Kernel**: 4.x or higher (for USB ACM driver)
- **Network**: Gigabit Ethernet adapter
- **USB**: USB 2.0 or higher

### Performance Notes
- LVDS data rate can exceed 1 Gbps depending on configuration
- Use a direct ethernet connection (not through a switch) for best performance
- Disable network power management on eth0:
  ```bash
  sudo ethtool -s eth0 wol d
  ```

## Support

For hardware-specific issues:
- TI E2E Forums: [mmWave Sensors Forum](https://e2e.ti.com/support/sensors/f/1023)
- DCA1000EVM User Guide: Available on TI website
- mmWave SDK Documentation: [TI mmWave SDK](https://www.ti.com/tool/MMWAVE-SDK)

For software/ROS issues:
- See `SETUP_README.md` for software configuration
- See `QUICK_START.md` for running the system
