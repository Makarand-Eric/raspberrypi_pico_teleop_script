import serial
import time

# Open the serial port
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

print("Serial port opened")

try:
    while True:
        command = "Hello Pico\n"
        ser.write(command.encode('utf-8'))
        print(f"Sent: {command.strip()}")
        time.sleep(2)  # Send data every 2 seconds
except KeyboardInterrupt:
    print("Exiting...")

ser.close()

