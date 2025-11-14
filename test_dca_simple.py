#!/usr/bin/env python3
"""Simple test to see if DCA1000 responds to any UDP command"""
import socket
import time

DCA_IP = '192.168.33.180'
DCA_CMD_PORT = 4096

# Simple RESET_FPGA command
RESET_CMD = b"\x5a\xa5\x01\x00\x00\x00\xaa\xee"

print(f"Testing DCA1000 communication at {DCA_IP}:{DCA_CMD_PORT}")
print("Make sure:")
print("  1. DCA1000 is powered ON")
print("  2. Ethernet cable is connected")
print("  3. Network is configured (192.168.33.30/24)")
print()

try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)
    
    print(f"Sending RESET_FPGA command...")
    sock.sendto(RESET_CMD, (DCA_IP, DCA_CMD_PORT))
    
    print("Waiting for response...")
    data, addr = sock.recvfrom(1024)
    
    print(f"✓ SUCCESS! DCA1000 responded!")
    print(f"  Response from: {addr}")
    print(f"  Response data: {data.hex()}")
    print(f"  Response length: {len(data)} bytes")
    
except socket.timeout:
    print("✗ TIMEOUT: No response from DCA1000")
    print("\nTroubleshooting:")
    print("  - Check DCA1000 power LED is ON")
    print("  - Verify ethernet cable connection")
    print("  - Confirm network: ip addr show eth0")
    print("  - Try ping: ping -c 2 192.168.33.180")
    
except Exception as e:
    print(f"✗ ERROR: {e}")
    
finally:
    sock.close()
