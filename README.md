# TI mmWave Radar ROS Integration

ROS Noetic integration for TI IWR1443 mmWave radar with DCA1000EVM data capture card.

## 📖 Documentation

**START HERE**: [INDEX.md](INDEX.md) - Complete documentation index

### Quick Links
- **[QUICK_START.md](QUICK_START.md)** - How to run the radar
- **[NETWORK_VERIFICATION.md](NETWORK_VERIFICATION.md)** - ⚠️ Critical: DCA1000 doesn't respond to ping!
- **[HARDWARE_REQUIREMENTS.md](HARDWARE_REQUIREMENTS.md)** - Hardware setup
- **[FIRMWARE_FLASHING_GUIDE.md](FIRMWARE_FLASHING_GUIDE.md)** - Firmware installation

## 🚀 Quick Start

```bash
# 1. Verify network (DON'T use ping - it won't work!)
ip neigh show | grep 192.168.33.180

# 2. Clean previous sessions
./cleanup_radar.sh

# 3. Run radar
./run_radar.sh
```

## ⚠️ Important Notes

1. **DCA1000 does NOT respond to ping** - Use `ip neigh show` to verify connection
2. **Always run `cleanup_radar.sh` first** - Prevents port conflicts
3. **SOP jumpers must be (0,0,0)** - Functional mode for normal operation
4. **Network must be 192.168.33.30/24** - Configure before starting radar

## 📚 Full Documentation

See **[INDEX.md](INDEX.md)** for complete documentation index.

## 🔧 System Requirements

- Ubuntu 20.04/22.04
- ROS Noetic
- Python 3.8+
- Nix package manager (for reproducible environment)

## 📦 Hardware

- TI IWR1443 mmWave Radar (AR-DevPack-EVM-012)
- DCA1000EVM Data Capture Card
- Ethernet cable (DCA1000 ↔ PC)
- USB cables (2x for radar and DCA1000)
- Power adapter for DCA1000

## 📄 License

See LICENSE file in iwr_raw_rosnode directory.

## 🙏 Credits

Based on [moodoki/iwr_raw_rosnode](https://github.com/moodoki/iwr_raw_rosnode)
