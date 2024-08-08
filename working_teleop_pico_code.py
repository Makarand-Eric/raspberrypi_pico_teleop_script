import select
import sys
import time
from machine import Pin, UART
from rmcs import RMCS  # Import the RMCS class from the rmcs.py file

# Set up the poll object
poll_obj = select.poll()
poll_obj.register(sys.stdin, select.POLLIN)

# Initialize the built-in LED (typically on GPIO 2 for Pico W)
led = Pin('LED', Pin.OUT)

# Initialize UART for motor controller (Assuming it's on UART1, change pins as needed)
rmcs1 = RMCS(1, 9600, 4, 5)  # Motor 1 on UART1 (change pins as needed)
slave_id1 = 7
pp_gain1 = 32
pi_gain1 = 16
vf_gain1 = 32
lpr1 = 2262
acceleration1 = 45
speed1 = 90
dir1 = 0

rmcs2 = RMCS(0, 9600, 12, 13)  # Motor 2 on UART0 (change pins as needed)
slave_id2 = 7
pp_gain2 = 32
pi_gain2 = 16
vf_gain2 = 32
lpr2 = 2262
acceleration2 = 45
speed2 = 90
dir2 =0

rmcs1.write_parameter(7,1,pp_gain1,pi_gain1,vf_gain1,lpr1,acceleration1,speed1)
rmcs2.write_parameter(7,1,pp_gain2,pi_gain2,vf_gain2,lpr2,acceleration2,speed2)


# Function to move the motors forward
def move_forward():
    rmcs1.speed(slave_id1, 100)  # Adjust speed as needed
    rmcs2.speed(slave_id2, 100)  # Adjust speed as needed
    dir1 =1
    dir2 =0
    rmcs1.enable_digital_mode(slave_id1, dir1)
    rmcs2.enable_digital_mode(slave_id2, dir2)

# Function to move the motors backward
def move_backward():
    rmcs1.speed(slave_id1, 100)  # Adjust speed as needed
    rmcs2.speed(slave_id2, 100)  # Adjust speed as needed
    dir1 =0
    dir2 =1 
    rmcs1.enable_digital_mode(slave_id1, dir1)
    rmcs2.enable_digital_mode(slave_id2, dir2)
    
def move_left():
    rmcs1.speed(slave_id1, 100)  # Adjust speed as needed
    rmcs2.speed(slave_id2, 100)  # Adjust speed as needed
    dir1 =1
    dir2 =1
    rmcs1.enable_digital_mode(slave_id1, dir1)
    rmcs2.enable_digital_mode(slave_id2, dir2)
    
def move_right():
    rmcs1.speed(slave_id1, 100)  # Adjust speed as needed
    rmcs2.speed(slave_id2, 100)  # Adjust speed as needed
    dir1 =0
    dir2 =0
    rmcs1.enable_digital_mode(slave_id1, dir1)
    rmcs2.enable_digital_mode(slave_id2, dir2)

# Function to stop the motors
def stop_motors(dir1,dir2):
    rmcs1.brake_motor(slave_id1,dir1)
    rmcs2.brake_motor(slave_id2,dir2)

# Open the file in append mode
with open('/data.txt', 'a') as file:
    # Loop indefinitely
    while True:
        # Wait for input on stdin
        poll_results = poll_obj.poll(1)  # the '1' is how long it will wait for message before looping again (in milliseconds)
        if poll_results:
            # Read the data from stdin (read data coming from PC)
            data = sys.stdin.readline().strip()
            # Write the data to stdout
            sys.stdout.write("received data: " + data + "\r")
            # Write the data to the file
            file.write(data + '\n')
            file.flush()  # Ensure data is written to the file
            # Blink the LED
            led.on()
            time.sleep(0.5)  # Keep the LED on for 0.5 seconds
            led.off()
            
            # Handle received commands
            if data == "forward":
                move_forward()
                dir1 =0
                dir2 =1
            elif data == "backward":
                move_backward()
                dir1 =1
                dir2 =0
            elif data == "left":
                move_left()
                dir1 =0
                dir2 =0
            elif data == "right":
                move_right()
                dir1 =1
                dir2 =1
            elif data == "stop":
                stop_motors(dir1,dir2)
        else:
            # do something if no message received (like feed a watchdog timer)
            continue


