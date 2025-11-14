#!/usr/bin/env python3
"""
Manually test sending sensorStart/sensorStop commands to the radar
"""
import serial
import time

def send_command(ser, cmd):
    """Send a command character by character with delays"""
    print(f"Sending: {cmd}")
    for char in cmd:
        ser.write(char.encode('utf-8'))
        time.sleep(0.010)  # 10ms delay between characters
    ser.write(b'\r')
    ser.reset_input_buffer()
    time.sleep(0.010)
    time.sleep(0.100)  # 100ms delay after command
    response = ser.read(size=100)
    print(f"Response: {response}")
    return response

def main():
    print("Opening serial port /dev/ttyACM0...")
    ser = serial.Serial(
        port='/dev/ttyACM0',
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.100
    )
    
    print("Serial port opened")
    print("")
    
    # Clear buffer
    for i in range(5):
        ser.write(b'\r')
        ser.reset_input_buffer()
        time.sleep(0.1)
    
    print("=" * 50)
    print("Sending sensorStart command...")
    print("=" * 50)
    resp = send_command(ser, "sensorStart")
    
    print("")
    print("Waiting 3 seconds...")
    time.sleep(3)
    
    print("")
    print("=" * 50)
    print("Sending sensorStop command...")
    print("=" * 50)
    resp = send_command(ser, "sensorStop")
    
    ser.close()
    print("\nDone!")

if __name__ == "__main__":
    main()
