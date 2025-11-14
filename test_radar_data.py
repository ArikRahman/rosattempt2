#!/usr/bin/env python3
"""
Simple test to verify radar data collection is working.
This script tests the data socket independently.
"""
import socket
import struct
import time

def test_data_reception():
    """Test if we're receiving data on port 4098"""
    print("Testing data reception on port 4098...")
    print("Make sure:")
    print("1. DCA1000 is powered on")
    print("2. Radar is configured and sensor started")
    print("")
    
    data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data_socket.bind(("192.168.33.30", 4098))
    data_socket.settimeout(5)  # 5 second timeout for testing
    
    packet_count = 0
    try:
        print("Listening for packets... (will timeout in 5 seconds if no data)")
        while packet_count < 10:
            try:
                msg, server = data_socket.recvfrom(2048)
                packet_count += 1
                seqn, bytec = struct.unpack('<IIxx', msg[:10])
                print(f"✓ Received packet {packet_count}: seqn={seqn}, bytes={bytec}, from={server}")
            except socket.timeout:
                print("⚠ No data received in 5 seconds")
                break
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        data_socket.close()
        
    if packet_count > 0:
        print(f"\n✓ SUCCESS: Received {packet_count} packets")
    else:
        print("\n✗ FAILED: No packets received")
        print("\nTroubleshooting:")
        print("- Check that DCA1000 is configured and armed")
        print("- Check that radar sensorStart command was sent")
        print("- Verify DCA1000 FPGA is configured with IP 192.168.33.180")
        
if __name__ == "__main__":
    test_data_reception()
