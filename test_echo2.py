import serial

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
ping_packet = bytearray([0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB])
ser.write(ping_packet)
response = ser.read(10)
print(f"Sent: {ping_packet.hex()}")
print(f"Received: {response.hex()}")
ser.close()
