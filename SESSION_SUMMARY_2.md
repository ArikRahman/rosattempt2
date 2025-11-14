# Session 2 Summary - Radar Data Collection Debugging

**Date:** November 13, 2025  
**Focus:** Getting radar to actually collect and publish data to ROS topics

---

## What We Accomplished

### 1. Fixed Socket Timeout Spam
**Problem:** The terminal was flooded with "timed out" messages  
**Root Cause:** The `collect_data()` function prints every socket timeout exception, which happens thousands of times per second due to the 25μs timeout  
**Solution:** Modified exception handling to silently ignore `socket.timeout` exceptions while still logging other errors

**File Modified:** `ws/src/mmWave/scripts/mmWave_class_noQt.py` lines 189-199
```python
# Before: printed every timeout
except Exception as e:
    print(e)
    return

# After: only log non-timeout exceptions  
except socket.timeout:
    # Expected when no data is available - this is normal
    return
except Exception as e:
    # Print other exceptions for debugging
    print(f"Data collection error: {e}")
    return
```

### 2. Created Debug Infrastructure
**Created Files:**
- `test_radar_data.py` - Tests data reception on port 4098 independently
- `test_sensor_start.py` - Tests serial commands to radar directly
- `cleanup_radar.sh` - Force cleanup of stuck processes and ports

**Added Debug Output:**
- Queue size monitoring in `check_and_publish_thread_func`
- Packet reception counters in `collect_data`
- Total published frame counter

### 3. Identified Core Issue: No Data Being Received

**Discovery Process:**
1. ✓ ROS topics exist (`/radar_data`, `/config_string`)
2. ✓ Radar node process is running (86.9% CPU usage)
3. ✓ Publishers registered on topics
4. ✗ **No data packets received on port 4098**

**Test Results:**
```bash
# test_radar_data.py output:
⚠ No data received in 5 seconds
✗ FAILED: No packets received
```

**Queue Debug Output:**
```
[DEBUG] Queue size: 0, Total published: 0
```
Consistently showing 0 - circular buffer queue never gets filled.

### 4. Discovered Process Management Issue

**Problem:** Processes frequently get stuck, requiring manual cleanup  
**Symptoms:**
- "Address already in use" on port 4096/4098
- Multiple node instances running simultaneously
- ROS master conflicts

**Why This Happens:**
- Python scripts don't always clean up UDP sockets on exit
- Ctrl+C doesn't always propagate to background threads
- Port remains bound for TIME_WAIT period even after process dies
- Multiple terminal sessions create orphaned processes

**Solution:** Created `cleanup_radar.sh` script to force clean state

---

## Key Realizations

### Understanding the Data Flow
```
┌─────────────┐  LVDS    ┌──────────────┐  UDP:4098  ┌─────────────┐
│   Radar     │────────→ │  DCA1000EVM  │──────────→ │  Host PC    │
│  (xWR14xx)  │  data    │              │   data     │  Port 4098  │
└─────────────┘          └──────────────┘            └─────────────┘
      ↑                         ↑                           │
      │ Serial                  │ UDP:4096                  │
      │ Commands                │ Commands                  │
      │                         │                           ↓
      └─────────────────────────┴───────────────────  collect_data()
                                                            │
                                                            ↓
                                                      Circular Buffer
                                                            │
                                                            ↓
                                                     ROS Topic /radar_data
```

### Critical Sequence for Data Collection
1. **DCA Setup** - Send SYSTEM_CONNECT, READ_FPGA_VERSION, CONFIG_FPGA_GEN, CONFIG_PACKET_DATA
2. **IWR Configuration** - Send all radar config commands (flushCfg, channelCfg, adcCfg, etc.)
3. **Arm DCA** - Send RECORD_START command (DCA1000 now listens for LVDS data)
4. **Start Sensor** - Send sensorStart command (radar begins transmitting on LVDS)
5. **Data Collection** - collect_data_thread continuously receives UDP packets on port 4098

### The Socket Timeout Design Pattern
The `25e-5` second (0.00025s) timeout is **intentional** - it makes the socket non-blocking:
- Data thread runs in tight loop calling `collect_data()`
- Most calls timeout immediately (no data ready)
- When data arrives, it's received within microseconds
- This allows ~4000 checks per second without blocking

**Important:** This means "timed out" is not an error - it's normal operation!

### Why Data Might Not Be Received

**Possible Causes We're Investigating:**
1. **Sensor not started** - sensorStart command not sent or failed
2. **DCA not forwarding** - DCA1000 FPGA not configured to forward data
3. **Timing issue** - Sensor starts before DCA is ready
4. **Configuration incomplete** - Radar config didn't complete successfully
5. **Hardware issue** - LVDS connection, power, or FPGA state

**Evidence So Far:**
- ✓ DCA1000 responds to commands (ports 4096)
- ✓ Radar responds to serial commands
- ✓ Network routing works (192.168.33.30 ↔ 192.168.33.180)
- ✗ No UDP packets received on port 4098 (data port)

---

## Files Modified This Session

1. **ws/src/mmWave/scripts/mmWave_class_noQt.py**
   - Lines 189-199: Fixed socket timeout exception handling
   - Lines 189-218: Added debug output for packet reception
   - Lines 34-40: Added packet counter tracking

2. **ws/src/mmWave/scripts/no_Qt.py**
   - Lines 34-40: Added queue size and publish counter debug output

3. **Created Files:**
   - `test_radar_data.py` - Standalone data reception test
   - `test_sensor_start.py` - Standalone serial command test  
   - `cleanup_radar.sh` - Process and port cleanup utility

---

## Current Status

### ✓ Working
- ROS Noetic environment with Python 3
- Build system (catkin_make)
- DCA1000 communication (port 4096 commands)
- Serial communication with radar
- ROS node starts and runs
- Topics are created and advertised
- Threads are running (high CPU usage confirms)

### ✗ Not Working  
- **No data packets received on port 4098**
- No frames published to `/radar_data` topic
- Circular buffer queue stays at 0

### 🔍 Currently Investigating
- Whether sensorStart command is actually being sent
- Whether radar is properly configured before sensor start
- DCA1000 FPGA data forwarding configuration
- Timing between arm_dca() and toggle_capture()

---

## Next Steps

### Immediate Debugging Plan
1. ✓ Add debug output to verify sensorStart is sent
2. ✓ Test serial commands manually with test_sensor_start.py
3. ⏳ Verify radar configuration completes successfully
4. ⏳ Check DCA1000 RECORD_START acknowledgment
5. ⏳ Monitor LVDS data with oscilloscope/logic analyzer (if available)
6. ⏳ Check DCA1000 status LEDs during operation

### Code Improvements Needed
1. Better error handling and status reporting
2. Add timeouts and retries for critical operations
3. Verify acknowledgments from DCA1000 commands
4. Add health check for data reception
5. Implement automatic recovery from stuck states

### Documentation Improvements
1. ✓ Created cleanup_radar.sh for process management
2. ⏳ Document expected LED states on DCA1000
3. ⏳ Add troubleshooting flowchart
4. ⏳ Document normal vs abnormal behavior

---

## Lessons Learned

### Process Management is Critical
The radar system involves multiple processes (roscore, radar node) and network resources (3 UDP/TCP ports). Without proper cleanup:
- Processes become zombies
- Ports stay bound
- Multiple instances conflict
- System becomes unreliable

**Solution:** Always use `cleanup_radar.sh` before starting fresh.

### Debug Output is Essential
The original code had minimal logging. Without knowing:
- Whether packets are received
- Queue sizes
- Frame counts
- Error conditions

...it's impossible to debug data flow issues.

### Socket Timeout != Error
The flooded "timed out" messages looked like errors but were actually normal operation. Understanding the non-blocking socket pattern was key to not being distracted by false positives.

### Hardware Dependencies are Hard to Debug
Unlike pure software, we can't easily inspect:
- LVDS data transmission
- DCA1000 internal state
- FPGA configuration status
- Physical layer issues

This requires systematic testing of each component.

---

## Tools Created

### cleanup_radar.sh
```bash
./cleanup_radar.sh
```
- Kills all ROS and radar processes
- Frees ports 4096, 4098, 11311
- Verifies cleanup succeeded
- Shows next steps

**When to use:**
- Before every fresh start
- When "Address already in use" error appears
- When node seems stuck or unresponsive
- After Ctrl+C doesn't fully stop processes

### test_radar_data.py
```bash
python3 test_radar_data.py
```
- Tests port 4098 data reception independently
- 5 second timeout per test
- Reports packet counts and statistics
- Useful for verifying DCA1000 is transmitting

### test_sensor_start.py
```bash
python3 test_sensor_start.py
```
- Tests serial commands directly
- Sends sensorStart/sensorStop
- Shows radar responses
- Useful for verifying radar configuration

---

## Questions Still Unanswered

1. **Why is no data received on port 4098?**
   - Is DCA1000 receiving LVDS data from radar?
   - Is DCA1000 FPGA configured to forward data?
   - Is the RECORD_START command actually arming the DCA?

2. **Is the sensor actually starting?**
   - Does sensorStart command succeed?
   - Is radar properly configured first?
   - What does radar serial output show?

3. **Is there a timing issue?**
   - Should we wait longer between arm_dca() and toggle_capture()?
   - Does DCA need time to initialize after RECORD_START?
   - Is there a race condition?

4. **What's the expected DCA1000 behavior?**
   - What LEDs should be on/blinking?
   - Does it need to be reconfigured after each stop?
   - How to verify FPGA is in correct state?

---

## References

- Previous session: `SESSION_SUMMARY.md`
- Hardware setup: `HARDWARE_REQUIREMENTS.md`
- Quick start: `QUICK_START.md`
- Setup guide: `SETUP_README.md`
- DCA1000 config: `configure_dca1000_simple.py`
- Original repo: https://github.com/moodoki/iwr_raw_rosnode
