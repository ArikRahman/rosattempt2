#!/usr/bin/env python3
"""
Test radar serial communication in functional mode.
This checks if the radar is responding to commands.
"""
import serial
import time

def test_serial():
    print("Testing radar serial communication...")
    print("Port: /dev/ttyACM0")
    print("Baudrate: 115200")
    print("")
    
    try:
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        time.sleep(0.1)
        
        # Send version command
        print("Sending 'version' command...")
        ser.write(b'version\r')
        time.sleep(0.3)
        response = ser.read(500)
        
        if response:
            print(f"✓ Radar responded:")
            print(response.decode('utf-8', errors='ignore'))
        else:
            print("✗ No response from radar")
            print("\nTroubleshooting:")
            print("- Check that radar is powered on")
            print("- Verify SOP jumpers are set correctly (SOP0=0, SOP1=0)")
            print("- Try power cycling the radar")
        
        ser.close()
        
    except serial.SerialException as e:
        print(f"✗ Serial port error: {e}")
        print("\nTroubleshooting:")
        print("- Check that /dev/ttyACM0 exists")
        print("- Verify USB connection")
        print("- Check serial permissions (dialout group)")

if __name__ == "__main__":
    test_serial()
