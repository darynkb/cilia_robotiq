import serial
import time

PORT = '/dev/ttyACM0'  
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)  
 
print("start reading...\n")
 
try:
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            print(line)  # 
except KeyboardInterrupt:
    print(" ")
finally:
    ser.close()