#!/usr/bin/env python3
"""
Simple DCA1000EVM Configuration Script
Based on mmwave-capture-std library
"""

import socket
import struct
import sys
import time

class DCA1000MagicNumber:
    MAGIC_HEADER = 0xA55A
    MAGIC_FOOTER = 0xEEAA

class DCA1000Command:
    RESET_FPGA = 1
    RESET_AR_DEV_CMD = 2
    CONFIG_FPGA_GEN = 3
    CONFIG_EEPROM = 4
    RECORD_START = 5
    RECORD_STOP = 6
    SYSTEM_CONNECTION = 9
    SYSTEM_ERROR_STATUS = 0xA
    CONFIG_PACKET_DELAY = 0xB
    READ_FPGA_VERSION = 0xE

class SimpleDCA1000:
    """Simplified DCA1000 configurator based on mmwave-capture-std"""
    
    def __init__(self, host_ip="192.168.33.30", dca_ip="192.168.33.180", 
                 config_port=4096, data_port=4098):
        self.host_ip = host_ip
        self.dca_ip = dca_ip
        self.config_port = config_port
        self.data_port = data_port
        
        # Create and bind socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.host_ip, self.config_port))
        self.sock.settimeout(3.0)
        
        print(f"Initialized: {host_ip}:{config_port} -> {dca_ip}:{config_port}")
    
    def _send_command(self, cmd_code, data=b"", return_raw_status=False):
        """Send command to DCA1000 and get response"""
        # Construct command packet
        cmd_header = struct.pack(
            "<HHH",
            DCA1000MagicNumber.MAGIC_HEADER,
            cmd_code,
            len(data),
        )
        cmd_footer = struct.pack("<H", DCA1000MagicNumber.MAGIC_FOOTER)
        cmd = cmd_header + data + cmd_footer
        
        # Send command
        self.sock.sendto(cmd, (self.dca_ip, self.config_port))
        
        # Receive response
        try:
            resp, addr = self.sock.recvfrom(1024)
        except socket.timeout:
            print(f"  ✗ Timeout waiting for response")
            return False if not return_raw_status else -1
        
        # Decode response
        resp_dec = struct.unpack("<HHHH", resp)
        
        # Verify response
        if resp_dec[0] != DCA1000MagicNumber.MAGIC_HEADER:
            print(f"  ✗ Invalid header: {resp_dec[0]:04x}")
            return False if not return_raw_status else -1
        
        if resp_dec[1] != cmd_code:
            print(f"  ✗ Wrong command code in response: {resp_dec[1]:04x}")
            return False if not return_raw_status else -1
        
        if resp_dec[3] != DCA1000MagicNumber.MAGIC_FOOTER:
            print(f"  ✗ Invalid footer: {resp_dec[3]:04x}")
            return False if not return_raw_status else -1
        
        # Return status
        if return_raw_status:
            return resp_dec[2]
        return resp_dec[2] == 0
    
    def system_connection(self):
        """Check if DCA1000 is connected"""
        print("Testing connection...")
        result = self._send_command(DCA1000Command.SYSTEM_CONNECTION)
        if result:
            print("  ✓ Connection successful")
        else:
            print("  ✗ Connection failed")
        return result
    
    def read_fpga_version(self):
        """Read FPGA version"""
        print("Reading FPGA version...")
        status = self._send_command(DCA1000Command.READ_FPGA_VERSION, 
                                    return_raw_status=True)
        if status == -1:
            print("  ✗ Failed to read version")
            return None
        
        major = status & 0x7F
        minor = (status >> 7) & 0x7F
        mode = (status & 0x4000) == 0x4000
        
        print(f"  ✓ FPGA Version: {major}.{minor}")
        print(f"  ✓ Playback Mode: {mode}")
        return (major, minor, mode)
    
    def config_fpga(self, lvds_mode=2, data_format_mode=3):
        """Configure FPGA general settings"""
        print("Configuring FPGA...")
        data = struct.pack(
            "<BBBBBB",
            1,  # data_logging_mode: 1=raw
            lvds_mode,  # lvdsMode: 2 for 4-lane
            1,  # data_transfer_mode: 1=LVDSCapture
            2,  # data_capture_mode: 2=ethernetStream
            data_format_mode,  # data_format_mode: 3
            30,  # Timer (fixed)
        )
        result = self._send_command(DCA1000Command.CONFIG_FPGA_GEN, data)
        if result:
            print("  ✓ FPGA configured")
        else:
            print("  ✗ FPGA configuration failed")
        return result
    
    def config_packet_delay(self, delay_us=5):
        """Configure packet delay"""
        print(f"Configuring packet delay to {delay_us}µs...")
        # Convert microseconds to FPGA clock cycles
        delay_cycles = (delay_us * 1000) // 8
        data = struct.pack("<HH", delay_cycles, 0)
        result = self._send_command(DCA1000Command.CONFIG_PACKET_DELAY, data)
        if result:
            print("  ✓ Packet delay configured")
        else:
            print("  ✗ Packet delay configuration failed")
        return result
    
    def config_eeprom(self, system_ip="192.168.33.30", dca_ip="192.168.33.180",
                     mac_address="12-34-56-78-90-12", config_port=4096, data_port=4098):
        """Write IP configuration to EEPROM (PERMANENT!)"""
        print("\n" + "="*60)
        print("WARNING: Writing to EEPROM (permanent configuration)")
        print(f"  System IP: {system_ip}")
        print(f"  DCA IP: {dca_ip}")
        print(f"  MAC: {mac_address}")
        print(f"  Config Port: {config_port}")
        print(f"  Data Port: {data_port}")
        print("="*60)
        
        # Parse IPs (note: reversed order in the data packet)
        sys_ip_parts = list(map(int, system_ip.split('.')))[::-1]
        dca_ip_parts = list(map(int, dca_ip.split('.')))[::-1]
        mac_parts = list(map(lambda x: int(x, 16), mac_address.split('-')))[::-1]
        
        # Build EEPROM data packet
        data = struct.pack(
            "<BBBBBBBBBBBBBBHH",
            *sys_ip_parts,  # System IP (4 bytes, reversed)
            *dca_ip_parts,  # DCA IP (4 bytes, reversed)
            *mac_parts,     # MAC address (6 bytes, reversed)
            config_port,    # Config port
            data_port,      # Data port
        )
        
        print(f"\nEEPROM data packet: {data.hex()}")
        result = self._send_command(DCA1000Command.CONFIG_EEPROM, data)
        
        if result:
            print("  ✓ EEPROM configured successfully")
            print("\n" + "="*60)
            print("NEXT STEPS:")
            print("1. Power cycle the DCA1000EVM (unplug and replug power)")
            print("2. Wait 10 seconds for it to boot")
            print("3. Test: ping 192.168.33.180")
            print("="*60)
        else:
            print("  ✗ EEPROM configuration failed")
        
        return result
    
    def reset_fpga(self):
        """Reset FPGA"""
        print("Resetting FPGA...")
        result = self._send_command(DCA1000Command.RESET_FPGA)
        if result:
            print("  ✓ FPGA reset")
        else:
            print("  ✗ FPGA reset failed")
        return result
    
    def close(self):
        """Close socket"""
        self.sock.close()


def main():
    print("="*60)
    print("DCA1000EVM Configuration Tool")
    print("Based on mmwave-capture-std library")
    print("="*60)
    print()
    
    # Check if we can ping the target IP first
    import subprocess
    result = subprocess.run(['ping', '-c', '1', '-W', '1', '192.168.33.180'],
                          capture_output=True)
    
    if result.returncode == 0:
        print("✓ DCA1000 already reachable at 192.168.33.180")
        print("  No EEPROM configuration needed!")
        print("\nYou can now run the radar system.")
        sys.exit(0)
    else:
        print("✗ DCA1000 not reachable at 192.168.33.180")
        print("  Will attempt to configure it...\n")
    
    try:
        # Initialize DCA1000
        dca = SimpleDCA1000()
        
        # Test connection
        if not dca.system_connection():
            print("\n❌ ERROR: Cannot connect to DCA1000!")
            print("\nPossible issues:")
            print("1. DCA1000 is not powered on")
            print("2. Ethernet cable not connected")
            print("3. DCA1000 might be at a different IP address")
            print("4. Your network interface doesn't have IP 192.168.33.30")
            print("\nTry checking:")
            print("  ip addr show eth0 | grep 192.168.33.30")
            sys.exit(1)
        
        print()
        
        # Read version
        version = dca.read_fpga_version()
        if not version:
            print("\n⚠ Warning: Could not read FPGA version, but will continue...")
        
        print()
        
        # Configure FPGA
        dca.config_fpga()
        time.sleep(0.5)
        
        print()
        
        # Configure packet delay
        dca.config_packet_delay(delay_us=25)
        time.sleep(0.5)
        
        print()
        
        # Ask before writing to EEPROM
        response = input("\nWrite configuration to EEPROM? This is PERMANENT! (yes/no): ")
        
        if response.lower() == 'yes':
            print()
            success = dca.config_eeprom()
            
            if success:
                print("\n✅ SUCCESS! Configuration written to EEPROM")
                print("\nDon't forget to power cycle the DCA1000!")
            else:
                print("\n❌ FAILED to write EEPROM")
                sys.exit(1)
        else:
            print("\nEEPROM write cancelled - configuration is temporary only")
            print("(Will be lost when DCA1000 is power cycled)")
        
        dca.close()
        
    except KeyboardInterrupt:
        print("\n\nCancelled by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
