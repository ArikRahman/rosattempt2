#!/usr/bin/env python3
"""
Configure DCA1000EVM IP address via UDP commands
Based on DCA1000EVM User Guide and cf.json configuration
"""

import socket
import struct
import time

class DCA1000Configurator:
    def __init__(self, current_dca_ip="192.168.33.180", system_ip="192.168.33.30"):
        self.dca_ip = current_dca_ip
        self.system_ip = system_ip
        self.cmd_port = 4096
        self.data_port = 4098
        
    def create_command(self, cmd_code, data=b''):
        """Create a DCA1000 command packet"""
        header = b'\x5a\xa5'  # Magic word
        length = len(data)
        footer = b'\xaa\xee'
        
        packet = header + struct.pack('<H', cmd_code) + struct.pack('<H', length) + data + footer
        return packet
    
    def send_command(self, cmd_code, data=b'', wait_response=True):
        """Send command to DCA1000 and optionally wait for response"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.system_ip, self.cmd_port))
        sock.settimeout(5)
        
        packet = self.create_command(cmd_code, data)
        print(f"Sending command {cmd_code:04x} to {self.dca_ip}:{self.cmd_port}")
        print(f"Packet: {packet.hex()}")
        
        sock.sendto(packet, (self.dca_ip, self.cmd_port))
        
        if wait_response:
            try:
                response, addr = sock.recvfrom(1024)
                print(f"Response from {addr}: {response.hex()}")
                return response
            except socket.timeout:
                print("No response received (timeout)")
                return None
        
        sock.close()
        return None
    
    def system_connect(self):
        """Connect to DCA1000 system"""
        print("\n=== Connecting to DCA1000 ===")
        # Command: 0x09 SYSTEM_CONNECT_CMD_CODE
        return self.send_command(0x09)
    
    def read_fpga_version(self):
        """Read FPGA version"""
        print("\n=== Reading FPGA Version ===")
        # Command: 0x0E READ_FPGA_VERSION_CMD_CODE
        return self.send_command(0x0E)
    
    def config_fpga_gen(self):
        """Configure FPGA general settings"""
        print("\n=== Configuring FPGA ===")
        # Command: 0x03 CONFIG_FPGA_GEN_CMD_CODE
        # Data: 6 bytes as per cf.json settings
        data = struct.pack('BBBBBB', 
            0x01,  # lvdsMode = 1
            0x01,  # dataTransferMode = 1
            0x01,  # dataFormatMode (will be overridden)
            0x02,  # dataLoggingMode  
            0x03,  # dataCaptureMode
            0x1e   # packetDelay = 30 us
        )
        return self.send_command(0x03, data)
    
    def config_packet_data(self):
        """Configure packet data"""
        print("\n=== Configuring Packet Data ===")
        # Command: 0x0B CONFIG_PACKET_DATA_CMD_CODE
        # Data format from the default command
        data = struct.pack('<HH', 1472, 2500) + b'\x00\x00'
        return self.send_command(0x0B, data)
    
    def config_eeprom(self, new_system_ip="192.168.33.30", 
                     new_dca_ip="192.168.33.180",
                     mac_address="12:34:56:78:90:12"):
        """
        Configure EEPROM with new IP addresses
        WARNING: This writes to EEPROM and changes persist after power cycle
        """
        print("\n=== Configuring EEPROM (PERMANENT) ===")
        print(f"System IP: {new_system_ip}")
        print(f"DCA IP: {new_dca_ip}")
        print(f"MAC: {mac_address}")
        
        # Command: 0x04 CONFIG_EEPROM_CMD_CODE
        # Parse IP addresses
        sys_ip_parts = [int(x) for x in new_system_ip.split('.')]
        dca_ip_parts = [int(x) for x in new_dca_ip.split('.')]
        mac_parts = [int(x, 16) for x in mac_address.split(':')]
        
        # Build data packet
        data = struct.pack('BBBB', *sys_ip_parts)  # System IP (4 bytes)
        data += struct.pack('BBBB', *dca_ip_parts)  # DCA IP (4 bytes)
        data += struct.pack('BBBBBB', *mac_parts)   # MAC address (6 bytes)
        data += struct.pack('<HH', self.cmd_port, self.data_port)  # Ports (4 bytes)
        
        print(f"EEPROM data: {data.hex()}")
        return self.send_command(0x04, data)
    
    def reset_fpga(self):
        """Reset FPGA"""
        print("\n=== Resetting FPGA ===")
        # Command: 0x01 RESET_FPGA_CMD_CODE
        return self.send_command(0x01, wait_response=False)
    
    def full_configuration(self):
        """Run full configuration sequence"""
        print("=" * 60)
        print("DCA1000EVM Configuration Utility")
        print("=" * 60)
        
        # Step 1: Connect
        resp = self.system_connect()
        if resp is None:
            print("\n❌ ERROR: Cannot connect to DCA1000EVM!")
            print("   Make sure:")
            print("   1. DCA1000 is powered on")
            print("   2. Ethernet cable is connected")
            print("   3. Your network interface has IP 192.168.33.30")
            print("   4. DCA1000 current IP is 192.168.33.180 (or change script)")
            return False
        
        time.sleep(0.5)
        
        # Step 2: Read version
        self.read_fpga_version()
        time.sleep(0.5)
        
        # Step 3: Configure FPGA
        self.config_fpga_gen()
        time.sleep(0.5)
        
        # Step 4: Configure packet data
        self.config_packet_data()
        time.sleep(0.5)
        
        # Step 5: Write to EEPROM (makes config permanent)
        print("\n" + "!" * 60)
        print("WARNING: About to write to EEPROM")
        print("This will make the IP configuration permanent!")
        print("!" * 60)
        response = input("Continue? (yes/no): ")
        
        if response.lower() == 'yes':
            self.config_eeprom()
            time.sleep(1)
            
            print("\n✓ Configuration written to EEPROM")
            print("\nNext steps:")
            print("1. Power cycle the DCA1000EVM (unplug and replug)")
            print("2. Wait 10 seconds")
            print("3. Test: ping 192.168.33.180")
            return True
        else:
            print("\nEEPROM write cancelled - configuration is temporary")
            return False


if __name__ == "__main__":
    import sys
    
    print("DCA1000EVM IP Configuration Tool")
    print("-" * 60)
    
    # Check if we can even reach the network
    import subprocess
    result = subprocess.run(['ping', '-c', '1', '-W', '1', '192.168.33.180'], 
                          capture_output=True)
    
    if result.returncode == 0:
        print("✓ DCA1000 already reachable at 192.168.33.180")
        print("  No configuration needed!")
        sys.exit(0)
    else:
        print("✗ DCA1000 not reachable at 192.168.33.180")
        print("  Will attempt configuration...")
    
    configurator = DCA1000Configurator()
    success = configurator.full_configuration()
    
    if success:
        print("\n" + "=" * 60)
        print("Configuration complete!")
        print("=" * 60)
    else:
        print("\n" + "=" * 60)
        print("Configuration failed or cancelled")
        print("=" * 60)
        sys.exit(1)
