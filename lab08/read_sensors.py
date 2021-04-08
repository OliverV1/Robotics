#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import json
import signal
import sys
import time

# This function will be called when CTRL+C is pressed
def signal_handler(sig, frame):
    print('\nYou pressed Ctrl+C! Closing the program nicely :)')
    try:
        ser.close()
    except Exception:
        pass
    sys.exit(0)

# Register a callback for CTRL+C
signal.signal(signal.SIGINT, signal_handler)


try:
    # Initialize serial connection
    ser = serial.Serial('/dev/ttyUSB0', 115200)
except Exception as e:
    print("Failed to establish a serial connection:")
    print(e)
    sys.exit(-1)

# Make sure Arduino is ready to send the data.
print("Syncing serial...0%\r", end='')
while ser.in_waiting == 0:
    ser.write("R".encode())
print("Syncing serial...50%\r", end='')
while ser.in_waiting > 0:
    ser.readline()
print("Syncing serial...100%")


while True:
    # Read the serial input to string
    ser.write("R".encode()) # Send something to the Arduino to indicate we're ready to get some data.
    serial_line = ser.readline().strip() # Read the data from serial.

    try:
        # Decode the received JSON data
        arduino_data = json.loads(serial_line.decode())

        # Extract the sensor values
        ls1 = arduino_data['ls1']
        ls2 = arduino_data['ls2']
        ls3 = arduino_data['ls3']
        ls4 = arduino_data['ls4']
        ls5 = arduino_data['ls5']
        dist = arduino_data['us1']

    except Exception as e:  # Something went wrong extracting the JSON.
        dist = -1           # Handle the situation.
        print(e)
        pass

    if dist != -1: # If a JSON was correctly extracted, continue.
        # Print received to the console
        print("LS1: ", ls1, "LS2: ", ls2, "LS3: ", ls3, "LS4: ", ls4, "LS5: ", ls5, "DIST: ", dist)

    # Throttle the loop to about 10 times per second
    time.sleep(.1)

