# Script Improvements Summary

## Problem

The `run_radar.sh` script had a critical flaw: when roscore was started in the background with `&`, it would die immediately because the parent nix shell exited.

**Symptoms**:
- roscore would start but die within seconds
- `rostopic list` would fail silently (no output)
- Radar node couldn't connect to ROS master
- Inconsistent behavior across sessions

## Root Cause

Original `run_radar.sh` code:
```bash
nix develop --command bash -c "roscore >/dev/null 2>&1 &"
```

**Problem**: The `nix develop` shell exits immediately after backgrounding roscore, which causes roscore to receive SIGHUP and terminate.

## Solution

Completely rewrote both `run_radar.sh` and `cleanup_radar.sh` with proper process management.

### New `run_radar.sh` Features

1. **Persistent roscore**
   - Starts roscore in background properly
   - Waits for roscore to be ready (polls port 11311)
   - Keeps roscore alive even after radar node exits
   - Tracks PID for monitoring

2. **Duplicate Instance Detection**
   - Checks if radar node is already running
   - Prevents port conflicts
   - Clear error message if duplicate detected

3. **Automatic Startup**
   - Starts roscore automatically if not running
   - Detects existing roscore and reuses it
   - No need for manual terminal juggling

4. **Graceful Shutdown**
   - Ctrl+C sends sensorStop to radar
   - Cleanly stops radar node
   - Leaves roscore running for faster restarts
   - Signal handlers for INT, TERM, EXIT

5. **Better User Feedback**
   - Clear progress messages
   - PID tracking for both processes
   - Instructions for monitoring topics
   - Formatted output with separators

### New `cleanup_radar.sh` Features

1. **Comprehensive Process Cleanup**
   - Kills: roscore, rosmaster, no_Qt.py, roslaunch, rosrun
   - Counts killed processes
   - Verifies all processes stopped

2. **Port Management**
   - Explicitly frees ports 4096, 4098, 11311
   - Uses proper protocol (UDP vs TCP)
   - Counts freed ports
   - Verifies ports are free

3. **Detailed Verification**
   - Lists remaining processes if any
   - Shows port status for each service
   - Color-coded output (✓/✗/⚠)
   - Clear success/warning status

4. **Better Error Handling**
   - Continues even if some kills fail
   - Waits for processes to die
   - Double-checks port status
   - Informative warnings if cleanup incomplete

## Code Changes

### run_radar.sh

**Before** (fragile, roscore dies):
```bash
echo "Checking for roscore..."
if ! timeout 2 bash -c "exec 3<>/dev/tcp/localhost/11311" 2>/dev/null; then
    echo "⚠️  WARNING: roscore is not running!"
    exit 1
fi
```

**After** (robust, auto-start):
```bash
start_roscore() {
    if ! pgrep -f "rosmaster" > /dev/null; then
        echo "[1/2] Starting roscore..."
        nix develop --command bash -c "source ws/devel/setup.bash && roscore" &
        ROSCORE_PID=$!
        
        # Wait for roscore to be ready
        for i in {1..30}; do
            if timeout 1 bash -c "exec 3<>/dev/tcp/localhost/11311" 2>/dev/null; then
                echo "      ✓ roscore ready on port 11311 (PID: $ROSCORE_PID)"
                return 0
            fi
            sleep 0.5
        done
        return 1
    fi
}
```

### cleanup_radar.sh

**Before** (basic, no verification):
```bash
echo "  Killing ROS processes..."
pkill -9 -f "roscore" 2>/dev/null
pkill -9 -f "rosmaster" 2>/dev/null
pkill -9 -f "no_Qt.py" 2>/dev/null
```

**After** (comprehensive, verified):
```bash
echo "[1/3] Stopping processes..."
KILLED=0

if pgrep -f "roscore" > /dev/null; then
    echo "      Killing roscore..."
    pkill -9 -f "roscore" 2>/dev/null && ((KILLED++))
fi

# ... (similar for other processes)

if [ $KILLED -eq 0 ]; then
    echo "      ✓ No processes to kill"
else
    echo "      ✓ Killed $KILLED process(es)"
fi
```

## Benefits

### Reliability
- ✅ roscore stays alive consistently
- ✅ No more mysterious "roscore died" issues
- ✅ Proper background process management
- ✅ Signal handling for clean shutdown

### User Experience
- ✅ Single command to start everything: `./run_radar.sh`
- ✅ Clear progress indicators
- ✅ PID tracking for debugging
- ✅ Helpful error messages
- ✅ No need for multiple terminals

### Debugging
- ✅ Can easily find PIDs
- ✅ Clear verification steps
- ✅ Detailed cleanup reporting
- ✅ Warning messages for edge cases

### Maintenance
- ✅ Well-documented code
- ✅ Modular functions
- ✅ Clear error handling
- ✅ Easy to extend

## Usage

### Before (Error-Prone)
```bash
# Terminal 1
roscore &  # Would die immediately!

# Terminal 2
./run_radar.sh  # Would fail: "roscore is not running"
```

### After (Simple and Reliable)
```bash
./cleanup_radar.sh  # Start clean
./run_radar.sh      # Starts both roscore and radar
# Ctrl+C to stop radar (roscore stays alive)
./cleanup_radar.sh  # Stop everything
```

## Testing

Both scripts were tested and verified:

1. **cleanup_radar.sh**:
   - ✅ Correctly identifies processes
   - ✅ Frees all ports
   - ✅ Verifies cleanup completion
   - ✅ Shows clear status messages

2. **run_radar.sh**:
   - ✅ roscore starts and stays alive (PID: 15066)
   - ✅ Radar node starts successfully (PID: 15244)
   - ✅ Detects duplicate instances
   - ✅ Graceful shutdown on Ctrl+C

## Files Modified

- `run_radar.sh` - Complete rewrite (72 lines → 116 lines)
- `cleanup_radar.sh` - Complete rewrite (63 lines → 94 lines)
- `QUICK_START.md` - Updated to reflect new workflow
- `SCRIPT_IMPROVEMENTS.md` - This documentation

## Migration Guide

No migration needed - scripts are backward compatible!

**Old workflow still works**:
```bash
./cleanup_radar.sh  # Same as before
./run_radar.sh      # Now better!
```

**New capabilities added**:
- Auto-start roscore
- Persistent roscore
- PID tracking
- Better error messages

---

**Date**: November 14, 2025  
**Issue**: roscore dying immediately when backgrounded  
**Resolution**: Proper process management with nix develop shells
