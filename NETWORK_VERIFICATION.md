# Network Verification Guide

## CRITICAL: DCA1000 Does NOT Respond to ICMP Ping!

**IMPORTANT**: The DCA1000EVM is an embedded device that **DOES NOT respond to ping (ICMP) requests**. 
A failed ping does NOT mean the device is disconnected!

## Correct Way to Verify DCA1000 Connection

### Method 1: Check ARP Table (FASTEST)
```bash
ip neigh show | grep 192.168.33.180
```

**Expected output:**
```
192.168.33.180 dev eth0 lladdr 12:34:56:78:90:12 REACHABLE
```

If you see `REACHABLE` or `STALE`, the DCA1000 **IS connected**.

### Method 2: Test UDP Communication (MOST RELIABLE)
```bash
timeout 5 python3 configure_dca1000_simple.py
```

**Expected output:**
```
✓ Connection successful
✓ FPGA Version: 2.8
✓ FPGA configured
```

### Method 3: Check Network Interface
```bash
ip addr show eth0 | grep "inet 192.168.33"
```

**Expected output:**
```
inet 192.168.33.30/24 brd 192.168.33.255 scope global eth0
```

## Common Mistakes to Avoid

### ❌ WRONG: Using ping to test DCA1000
```bash
ping 192.168.33.180  # This will ALWAYS fail even when DCA1000 is working!
```

### ✅ CORRECT: Using ARP or UDP communication
```bash
ip neigh show | grep 192.168.33.180  # Check ARP table
python3 configure_dca1000_simple.py   # Test actual communication
```

## Network Configuration

### Host Computer (Your PC)
- **Interface**: eth0
- **IP Address**: 192.168.33.30
- **Netmask**: 255.255.255.0 (/24)
- **Network**: 192.168.33.0/24

### DCA1000EVM
- **IP Address**: 192.168.33.180 (factory default, hardcoded)
- **MAC Address**: 12:34:56:78:90:12
- **Config Port**: UDP 4096
- **Data Port**: UDP 4098
- **Note**: Does NOT respond to ping!

## Troubleshooting

### Issue: "ip neigh show" returns nothing for 192.168.33.180

**Cause**: No ARP entry yet (device hasn't been contacted)

**Solution**: Try sending a UDP packet first
```bash
python3 configure_dca1000_simple.py
# Then check again
ip neigh show | grep 192.168.33.180
```

### Issue: Network interface eth0 has no IP

**Solution**: Configure it
```bash
sudo ip addr add 192.168.33.30/24 dev eth0
sudo ip link set eth0 up
```

### Issue: "Network is unreachable"

**Cause**: eth0 interface is down

**Solution**:
```bash
sudo ip link set eth0 up
```

## Quick Verification Checklist

Run these commands in order:

```bash
# 1. Check ethernet interface is up and configured
ip addr show eth0

# 2. Check ARP table for DCA1000
ip neigh show | grep 192.168.33.180

# 3. If no ARP entry, test UDP communication
timeout 5 python3 configure_dca1000_simple.py

# 4. Check ARP table again
ip neigh show | grep 192.168.33.180
```

**All good if:**
- eth0 shows `inet 192.168.33.30/24`
- ARP shows `192.168.33.180 ... REACHABLE` or `STALE`
- configure_dca1000_simple.py shows `✓ Connection successful`

## Why Doesn't DCA1000 Respond to Ping?

The DCA1000EVM runs minimal embedded firmware focused on:
- UDP communication on ports 4096 (config) and 4098 (data)
- FPGA configuration
- High-speed data capture

It does **NOT** implement:
- ICMP (ping) protocol
- TCP
- HTTP/web interface
- SSH or telnet

This is normal and expected behavior for embedded data acquisition devices.
