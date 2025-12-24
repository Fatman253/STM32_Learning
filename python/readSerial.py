import serial
import struct

ser = serial.Serial('COM13', 115200) 

while True:
    # 1. CHANGE SIZE: Read 9 bytes now (Old 7 + New 2)
    data = ser.read(9) 
    
    if len(data) == 9:
        # 2. UPDATE FORMAT: Add 'h' at the end for int16_t
        # '<' = Little Endian
        # 'I' = uint32 (Timestamp)
        # 'H' = uint16 (PWM)
        # 'B' = uint8  (Status)
        # 'h' = int16  (Encoder - Lowercase 'h' means SIGNED short)
        timestamp, pwm, status, encoder = struct.unpack('<IHBh', data)
        
        print(f"Time: {timestamp}, PWM: {pwm}, Status: {status}, Encoder: {encoder}")