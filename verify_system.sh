#!/bin/bash

# System Verification Script for TI mmWave Radar
# This script checks all prerequisites before running the radar

echo "============================================================"
echo "TI mmWave Radar System Verification"
echo "============================================================"
echo ""

ERRORS=0
WARNINGS=0

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test 1: Network Interface
echo "[1/7] Checking network interface..."
if ip addr show eth0 | grep -q "192.168.33.30/24"; then
    echo -e "${GREEN}✓${NC} Network configured: 192.168.33.30/24 on eth0"
else
    echo -e "${RED}✗${NC} Network NOT configured"
    echo "      Run: sudo ip addr add 192.168.33.30/24 dev eth0"
    ((ERRORS++))
fi
echo ""

# Test 2: DCA1000 Connectivity (ARP - NOT ping!)
echo "[2/7] Checking DCA1000 connectivity..."
echo "      ⚠️  NOTE: DCA1000 does NOT respond to ping!"
echo "      Using ARP table instead..."
ARP_RESULT=$(ip neigh show | grep 192.168.33.180)
if [[ -n "$ARP_RESULT" ]]; then
    if echo "$ARP_RESULT" | grep -q "REACHABLE\|STALE"; then
        echo -e "${GREEN}✓${NC} DCA1000 reachable: $ARP_RESULT"
    else
        echo -e "${YELLOW}⚠${NC} DCA1000 in ARP table but not recently contacted"
        echo "      $ARP_RESULT"
        ((WARNINGS++))
    fi
else
    echo -e "${YELLOW}⚠${NC} DCA1000 not in ARP table yet"
    echo "      This is normal if you haven't contacted it yet"
    echo "      Will test UDP communication next..."
    ((WARNINGS++))
fi
echo ""

# Test 3: DCA1000 UDP Communication
echo "[3/7] Testing DCA1000 UDP communication..."
if command -v python3 &> /dev/null; then
    if [[ -f "configure_dca1000_simple.py" ]]; then
        # Run with timeout and capture output
        OUTPUT=$(timeout 5 python3 configure_dca1000_simple.py <<< "no" 2>&1)
        if echo "$OUTPUT" | grep -q "Connection successful"; then
            echo -e "${GREEN}✓${NC} DCA1000 UDP communication working"
            FPGA_VERSION=$(echo "$OUTPUT" | grep "FPGA Version" | awk '{print $NF}')
            if [[ -n "$FPGA_VERSION" ]]; then
                echo "      FPGA Version: $FPGA_VERSION"
            fi
        else
            echo -e "${RED}✗${NC} DCA1000 not responding to UDP commands"
            ((ERRORS++))
        fi
    else
        echo -e "${YELLOW}⚠${NC} configure_dca1000_simple.py not found, skipping"
        ((WARNINGS++))
    fi
else
    echo -e "${YELLOW}⚠${NC} python3 not found, skipping"
    ((WARNINGS++))
fi
echo ""

# Test 4: Serial Port
echo "[4/7] Checking radar serial port..."
if [[ -e "/dev/ttyACM0" ]]; then
    if [[ -r "/dev/ttyACM0" && -w "/dev/ttyACM0" ]]; then
        echo -e "${GREEN}✓${NC} Serial port /dev/ttyACM0 accessible"
    else
        echo -e "${YELLOW}⚠${NC} Serial port exists but may need permissions"
        echo "      Run: sudo chmod 666 /dev/ttyACM0"
        echo "      Or add user to dialout group: sudo usermod -a -G dialout $USER"
        ((WARNINGS++))
    fi
else
    echo -e "${RED}✗${NC} Serial port /dev/ttyACM0 not found"
    echo "      Is the radar connected via USB?"
    ((ERRORS++))
fi
echo ""

# Test 5: Port Availability
echo "[5/7] Checking port availability..."
PORTS_IN_USE=""
for PORT in 4096 4098 11311; do
    if sudo lsof -i :$PORT &> /dev/null; then
        PORTS_IN_USE="$PORTS_IN_USE $PORT"
    fi
done

if [[ -z "$PORTS_IN_USE" ]]; then
    echo -e "${GREEN}✓${NC} Ports 4096, 4098, 11311 are available"
else
    echo -e "${YELLOW}⚠${NC} Ports in use:$PORTS_IN_USE"
    echo "      Run: ./cleanup_radar.sh"
    ((WARNINGS++))
fi
echo ""

# Test 6: ROS Environment
echo "[6/7] Checking ROS environment..."
if command -v nix &> /dev/null; then
    echo -e "${GREEN}✓${NC} Nix package manager found"
else
    echo -e "${RED}✗${NC} Nix package manager not found"
    ((ERRORS++))
fi
echo ""

# Test 7: Firmware Status
echo "[7/7] Checking firmware..."
if [[ -f "iwr_raw_rosnode/firmware/lvds_stream/xwr14xx/xwr14xx_lvds_stream.bin" ]]; then
    echo -e "${GREEN}✓${NC} LVDS streaming firmware binary found"
    echo "      Make sure this is flashed to the radar"
    echo "      See FIRMWARE_FLASHING_GUIDE.md for instructions"
else
    echo -e "${YELLOW}⚠${NC} LVDS firmware binary not found in expected location"
    ((WARNINGS++))
fi
echo ""

# Summary
echo "============================================================"
echo "Summary"
echo "============================================================"
if [[ $ERRORS -eq 0 && $WARNINGS -eq 0 ]]; then
    echo -e "${GREEN}✓ All checks passed!${NC} System ready to run."
    echo ""
    echo "Next steps:"
    echo "  1. ./cleanup_radar.sh"
    echo "  2. ./run_radar.sh"
elif [[ $ERRORS -eq 0 ]]; then
    echo -e "${YELLOW}⚠ $WARNINGS warning(s) found${NC}"
    echo "System may work, but review warnings above."
else
    echo -e "${RED}✗ $ERRORS error(s) and $WARNINGS warning(s) found${NC}"
    echo "Fix errors before running radar."
    exit 1
fi
echo ""

# Important reminders
echo "============================================================"
echo "Important Reminders"
echo "============================================================"
echo "• DCA1000 does NOT respond to ping - this is normal!"
echo "• Always run ./cleanup_radar.sh before starting"
echo "• See INDEX.md for complete documentation"
echo "• See NETWORK_VERIFICATION.md for network troubleshooting"
echo "============================================================"
