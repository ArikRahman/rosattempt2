# TI mmWave Radar ROS Integration - Documentation Index

## 📚 Quick Reference

### Getting Started (Read These First)
1. **[INDEX.md](INDEX.md)** ← You are here
2. **[HARDWARE_REQUIREMENTS.md](HARDWARE_REQUIREMENTS.md)** - Required hardware and connections
3. **[QUICK_START.md](QUICK_START.md)** - Step-by-step guide to run the radar

### Critical Knowledge
4. **[NETWORK_VERIFICATION.md](NETWORK_VERIFICATION.md)** - ⚠️ **READ THIS!** DCA1000 doesn't respond to ping!
5. **[HARDWARE_CHECKLIST.md](HARDWARE_CHECKLIST.md)** - Physical connection verification

### Setup & Configuration
6. **[SETUP_README.md](SETUP_README.md)** - Initial environment setup
7. **[FIRMWARE_FLASHING_GUIDE.md](FIRMWARE_FLASHING_GUIDE.md)** - How to flash LVDS streaming firmware

### Session Notes & Troubleshooting
8. **[SESSION_SUMMARY.md](SESSION_SUMMARY.md)** - Session 1 debugging notes
9. **[SESSION_SUMMARY_2.md](SESSION_SUMMARY_2.md)** - Session 2 debugging notes
10. **[SCRIPT_IMPROVEMENTS.md](SCRIPT_IMPROVEMENTS.md)** - roscore management fixes
11. **[DOCUMENTATION_UPDATE_SUMMARY.md](DOCUMENTATION_UPDATE_SUMMARY.md)** - Network verification documentation

---

## 🚀 Quick Start (3 Steps)

### Step 0: Verify System (Recommended)
```bash
cd /home/arik/rosattempt2
./verify_system.sh
```

This checks:
- Network configuration (192.168.33.30/24)
- DCA1000 connectivity (**using ARP, not ping!**)
- DCA1000 UDP communication
- Serial port access
- Port availability
- ROS environment

### Step 1: Verify Hardware Connections
```bash
# Check network is configured
ip addr show eth0 | grep 192.168.33

# Verify DCA1000 is reachable (NOT ping!)
ip neigh show | grep 192.168.33.180
```

**See**: [NETWORK_VERIFICATION.md](NETWORK_VERIFICATION.md) for details

### Step 2: Clean Start
```bash
cd /home/arik/rosattempt2
./cleanup_radar.sh
```

### Step 3: Run Radar
```bash
./run_radar.sh
```

**Expected**: Queue size increasing, ROS topic publishing data

---

## 🔧 Hardware Setup

### Required Components
- TI IWR1443 mmWave Radar (AR-DevPack-EVM-012)
- DCA1000EVM Data Capture Card
- USB cables (2x): Radar ↔ PC, DCA1000 ↔ PC
- Ethernet cable: DCA1000 ↔ PC
- Power adapter for DCA1000
- LVDS cable: Radar ↔ DCA1000

**See**: [HARDWARE_REQUIREMENTS.md](HARDWARE_REQUIREMENTS.md) for complete details

### SOP Jumper Settings

| Mode | SOP0 | SOP1 | SOP2 | Use Case |
|------|------|------|------|----------|
| **Functional** | 0 | 0 | 0 | Normal operation (boot from flash) |
| **Development** | 1 | 1 | 0 | Firmware flashing via UniFlash |

**Current Setup**: Functional Mode (0,0,0) - LVDS firmware in flash

---

## 📡 Network Configuration

### Host Computer
- Interface: `eth0`
- IP: `192.168.33.30/24`
- Check: `ip addr show eth0`

### DCA1000EVM
- IP: `192.168.33.180` (factory default, hardcoded)
- Config Port: UDP `4096`
- Data Port: UDP `4098`
- ⚠️ **Does NOT respond to ping!** Use `ip neigh show` instead

**See**: [NETWORK_VERIFICATION.md](NETWORK_VERIFICATION.md) - **READ THIS FIRST!**

---

## 🐛 Common Issues & Solutions

### Issue: "Queue size: 0" - No Data Received

**Diagnosis Steps**:
1. ✅ Verify DCA1000 connection (use ARP, not ping!)
2. ✅ Check LVDS firmware is flashed
3. ✅ Verify SOP jumpers in Functional Mode (0,0,0)
4. ✅ Check testFmkCfg parameters are correct
5. ✅ Ensure ports 4096/4098 are open

**See**: [SESSION_SUMMARY_2.md](SESSION_SUMMARY_2.md) for detailed debugging history

### Issue: "DCA1000 not reachable" (but it actually is!)

**Cause**: Tried to ping the DCA1000 (it doesn't respond to ping!)

**Solution**: Use ARP table instead
```bash
ip neigh show | grep 192.168.33.180
# If shows REACHABLE or STALE → DCA1000 IS connected!
```

**See**: [NETWORK_VERIFICATION.md](NETWORK_VERIFICATION.md)

### Issue: Network interface not configured

**Solution**:
```bash
sudo ip addr add 192.168.33.30/24 dev eth0
sudo ip link set eth0 up
```

### Issue: Stuck processes or ports in use

**Solution**:
```bash
./cleanup_radar.sh
```

---

## 📝 Key Files & Scripts

### Utility Scripts
- **`verify_system.sh`** - Comprehensive system check (**run this first!**)
- **`cleanup_radar.sh`** - Kill stuck processes, free ports
- **`run_radar.sh`** - Start roscore and radar node
- **`configure_dca1000_simple.py`** - Test DCA1000 connection and configuration
- **`test_radar_data.py`** - Test UDP data reception on port 4098
- **`test_radar_serial.py`** - Test radar serial communication
- **`check_radar_firmware.py`** - Verify firmware responds to commands

### Configuration Files
- **`ws/src/mmWave/scripts/configs/14xx/indoor_human_rcs.cfg`** - Radar configuration
- **`ws/src/mmWave/scripts/radar_config.py`** - Config parser (has testFmkCfg parameters)
- **`cf.json`** - DCA1000 configuration JSON

### Core Code
- **`ws/src/mmWave/scripts/mmWave_class_noQt.py`** - Main sensor interface
- **`ws/src/mmWave/scripts/no_Qt.py`** - ROS node entry point

---

## ⚙️ Important Configuration Values

### DCA1000 FPGA Configuration
- **lvdsMode**: `2` (critical - must be 2, not 1)
- **packetDelay**: `25` (microseconds)
- **FPGA Version**: Should report `2.8`

### Test Framework Configuration
- **testFmkCfg**: `0 0 1 1` (enables LVDS streaming)
  - Param 3 must be `1` (not 0)
  - Hardcoded in `radar_config.py` line 126

### Serial Communication
- **Port**: `/dev/ttyACM0`
- **Baud**: `115200`
- **Prompt**: `LVDS Stream:/>`

---

## 🔍 Verification Commands

### Check Everything is Working
```bash
# 1. Network configured
ip addr show eth0 | grep "192.168.33.30"

# 2. DCA1000 reachable (ARP, not ping!)
ip neigh show | grep "192.168.33.180"

# 3. Test DCA1000 communication
timeout 5 python3 configure_dca1000_simple.py

# 4. Ports open
sudo lsof -i :4096,4098

# 5. Radar serial accessible
ls -l /dev/ttyACM0
```

### Monitor Running Radar
```bash
# Check ROS topics
rostopic list

# Monitor data publication rate
rostopic hz /radar_data

# Echo data (will be binary)
rostopic echo /radar_data
```

---

## 📖 Detailed Documentation

### Hardware
- [HARDWARE_REQUIREMENTS.md](HARDWARE_REQUIREMENTS.md) - Complete hardware list
- [HARDWARE_CHECKLIST.md](HARDWARE_CHECKLIST.md) - Physical connection verification
- [NETWORK_VERIFICATION.md](NETWORK_VERIFICATION.md) - **Network troubleshooting (DCA1000 doesn't ping!)**

### Firmware
- [FIRMWARE_FLASHING_GUIDE.md](FIRMWARE_FLASHING_GUIDE.md) - Flash LVDS streaming firmware
- `iwr_raw_rosnode/firmware/` - Firmware source code

### Configuration
- [QUICK_START.md](QUICK_START.md) - Runtime configuration guide
- `iwr_raw_rosnode/radar_configs/` - Example radar configurations

### Development
- [SESSION_SUMMARY.md](SESSION_SUMMARY.md) - Session 1 notes
- [SESSION_SUMMARY_2.md](SESSION_SUMMARY_2.md) - Session 2 notes
- [SETUP_README.md](SETUP_README.md) - Environment setup

---

## 🎯 Current System Status

### ✅ Completed
- LVDS streaming firmware flashed to IWR1443
- Network configured (192.168.33.30/24 on eth0)
- DCA1000 verified reachable (via ARP)
- Socket timeout handling fixed
- DCA1000 FPGA configuration corrected (lvdsMode=2)
- FPGA reset added to initialization
- Test framework configuration fixed (testFmkCfg 0 0 1 1)
- Process cleanup utility created

### ⏳ In Progress
- Testing complete data pipeline
- Verifying UDP data reception on port 4098
- Confirming ROS topic publication

### 📋 Known Issues
- Queue size showing 0 (no UDP packets received yet)
- Need to verify LVDS hardware connection
- May need to adjust test framework parameters

---

## 🆘 Getting Help

### Debugging Process
1. Read [NETWORK_VERIFICATION.md](NETWORK_VERIFICATION.md) first!
2. Run `./cleanup_radar.sh`
3. Check [HARDWARE_CHECKLIST.md](HARDWARE_CHECKLIST.md)
4. Review recent session notes: [SESSION_SUMMARY_2.md](SESSION_SUMMARY_2.md)
5. Test individual components with utility scripts

### Useful References
- TI IWR1443 User Guide
- DCA1000EVM User Guide
- mmWave SDK Documentation
- Original repo: https://github.com/moodoki/iwr_raw_rosnode

---

## 📌 Critical Reminders

### ⚠️ ALWAYS Remember:
1. **DCA1000 does NOT respond to ping!** Use `ip neigh show` instead
2. **Run `./cleanup_radar.sh` before starting** - prevents port conflicts
3. **SOP jumpers must be (0,0,0) for normal operation** - Functional Mode
4. **Network must be configured BEFORE starting radar** - 192.168.33.30/24
5. **testFmkCfg must be "0 0 1 1"** - enables LVDS test framework

### 🔐 Don't Assume:
- ❌ Don't assume ping failure = device offline
- ❌ Don't assume radar is configured if you just flashed firmware
- ❌ Don't assume ports are free without checking
- ✅ Always verify with proper tools (ARP, UDP test, lsof)

---

**Last Updated**: November 14, 2025  
**System**: ROS Noetic, Python 3.12.11, Nix development environment  
**Hardware**: TI IWR1443 + DCA1000EVM
