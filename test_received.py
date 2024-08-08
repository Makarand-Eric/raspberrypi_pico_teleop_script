import select
import sys
import time
from machine import Pin

# Set up the poll object
poll_obj = select.poll()
poll_obj.register(sys.stdin, select.POLLIN)

# Initialize the built-in LED (typically on GPIO 2 for Pico W)
led = Pin('LED', Pin.OUT)

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
        else:
            # do something if no message received (like feed a watchdog timer)
            continue


