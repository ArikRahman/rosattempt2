#!/usr/bin/env python3
"""
Check radar firmware version and capability.
"""
import serial
import time

def check_firmware():
    print("Checking radar firmware...")
    print("")
    
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    time.sleep(0.5)
    
    # Clear any existing data
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    
    # Send a few carriage returns to get to prompt
    for i in range(3):
        ser.write(b'\r')
        time.sleep(0.1)
    
    ser.reset_input_buffer()
    
    # Send version command
    print("Sending 'version' command...")
    ser.write(b'version\r')
    time.sleep(0.5)
    response = ser.read(1000)
    
    if response:
        print("Response:")
        print(response.decode('utf-8', errors='ignore'))
        print("")
    
    # Check if sensorStart is a valid command
    ser.reset_input_buffer()
    print("Testing 'sensorStart' command...")
    ser.write(b'sensorStart\r')
    time.sleep(0.3)
    response2 = ser.read(500)
    
    if response2:
        resp_str = response2.decode('utf-8', errors='ignore')
        print("Response:")
        print(resp_str)
        
        if 'Done' in resp_str or 'done' in resp_str:
            print("✓ sensorStart command recognized")
        elif 'error' in resp_str.lower() or 'invalid' in resp_str.lower():
            print("✗ sensorStart not recognized - wrong firmware?")
        else:
            print("? Unknown response")
    
    ser.close()

if __name__ == "__main__":
    check_firmware()
