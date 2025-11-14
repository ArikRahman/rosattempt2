# Hardware Checklist for IWR1443 + DCA1000 System

## ❌ **CURRENT ISSUE: DCA1000 NOT RESPONDING**

The system is stuck because the DCA1000 data capture card is not powered on or not responding to network commands.

## Required Hardware Checks

### 1. DCA1000 Power
- [ ] **DCA1000 power supply connected** (5V adapter or USB power)
- [ ] **Power LED is ON** (look for illuminated LED on DCA1000 board)
- [ ] **DCA1000 switches** - Check DIP switch settings (consult DCA1000 manual)

### 2. IWR1443 Radar Power
- [ ] **Radar powered ON** (USB connection or 5V supply)
- [ ] **Power LED visible** on radar board
- [ ] **SOP jumpers set correctly**:
  - **Functional Mode** (normal operation): SOP0=0, SOP1=0, SOP2=0
  - Development Mode (flashing): SOP0=1, SOP1=1, SOP2=0

### 3. Physical Connections
- [ ] **Ethernet cable** connected from computer to DCA1000 RJ45 port
- [ ] **LVDS cable** connected between IWR1443 and DCA1000
  - 60-pin LVDS connector properly seated
  - Check for bent pins or misalignment
- [ ] **USB cable** from IWR1443 to computer (for serial CLI)
  - Shows up as `/dev/ttyACM0` when connected

### 4. Network Configuration
- [ ] **Ethernet interface UP**:
  ```bash
  ip link show eth0  # Should show "state UP"
  ```
- [ ] **IP address configured**:
  ```bash
  ip addr show eth0 | grep "192.168.33.30"
  ```
  Expected: `inet 192.168.33.30/24`
  
- [ ] **DCA1000 reachable**:
  ```bash
  ping -c 2 192.168.33.180
  ```
  Should get replies, not 100% packet loss

### 5. Test Commands

Run these to verify each component:

```bash
# Test 1: Network configured?
ip addr show eth0 | grep inet

# Test 2: DCA1000 reachable?
ping -c 2 192.168.33.180

# Test 3: DCA1000 responds to UDP?
python3 test_dca_simple.py

# Test 4: Radar serial port exists?
ls -l /dev/ttyACM0

# Test 5: Radar responds to serial?
python3 test_radar_serial.py
```

## Expected Results When Working

✅ DCA1000 power LED: **ON**  
✅ Radar power LED: **ON**  
✅ Ping 192.168.33.180: **replies received**  
✅ test_dca_simple.py: **"SUCCESS! DCA1000 responded!"**  
✅ test_radar_serial.py: **"LVDS Stream:/>" prompt visible**  

## Current Status

```
Network: ✅ 192.168.33.30/24 configured on eth0
Radar Serial: ✅ /dev/ttyACM0 accessible
DCA1000 Ping: ❌ 100% packet loss
DCA1000 UDP: ❌ No response to RESET_FPGA command
```

**ACTION REQUIRED: Check DCA1000 power and connections!**

The software is correctly configured, but the DCA1000 hardware is not powered or not communicating.
