import socket, struct, time

print('Testing DCA1000 connection...')
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('192.168.33.30', 4096))
sock.settimeout(3)

# SYSTEM_CONNECT
cmd = struct.pack('<HHH', 0xA55A, 0x09, 0x00) + struct.pack('<H', 0xEEAA)
print('Sending SYSTEM_CONNECT...')
sock.sendto(cmd, ('192.168.33.180', 4096))

try:
    resp, addr = sock.recvfrom(1024)
    print(f'✓ DCA1000 responded: {resp.hex()}')
    
    # Read FPGA version
    time.sleep(0.5)
    cmd = struct.pack('<HHH', 0xA55A, 0x0E, 0x00) + struct.pack('<H', 0xEEAA)
    print('Reading FPGA version...')
    sock.sendto(cmd, ('192.168.33.180', 4096))
    resp, addr = sock.recvfrom(1024)
    status = struct.unpack('<HHHH', resp)[2]
    major = status & 0x7F
    minor = (status >> 7) & 0x7F
    print(f'✓ FPGA Version: {major}.{minor}')
    print('✓ DCA1000 is working!')
    
except socket.timeout:
    print('✗ DCA1000 not responding')
    print('Check: Power, ethernet cable, IP configuration')

sock.close()
