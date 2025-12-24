import serial
import struct

ser = serial.Serial('COM5', 115200) # Replace COMx

while True:
    # Read exactly 7 bytes (size of your struct)
    data = ser.read(7) 
    
    # Unpack: '<' = Little Endian (STM32 standard)
    # 'I' = uint32 (4 bytes), 'H' = uint16 (2 bytes), 'B' = uint8 (1 byte)
    timestamp, pwm, status = struct.unpack('<IHB', data)
    
    print(f"Time: {timestamp}, PWM: {pwm}, Status: {status}")