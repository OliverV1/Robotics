#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import json
import signal
import sys
import time
import easygopigo3 as go

myRobot = go.EasyGoPiGo3()
# Retrieve data from ultrasonic sensor and line following module connected to Arduino
def getDataFromArduino(ser):
    data = {}
    ser.write("R".encode()) # Request data

    # Wait for a response to reach us from Arduino
    while not ser.in_waiting:
        pass

    # Read and parse the serial buffer
    while ser.in_waiting > 0:
        serial_line = ser.readline().strip()
        try:
            data = json.loads(serial_line.decode())
        except Exception as e:
            data = {}
            print(e)
    return data


# This function will be called when CTRL+C is pressed
def signal_handler(sig, frame):
    print('\nYou pressed Ctrl+C! Closing the program nicely :)')
    try:
        robot.stop()
        ser.close()
    except Exception:
        pass
    sys.exit(0)


# Register a callback for CTRL+C
signal.signal(signal.SIGINT, signal_handler)

#Create an instance of the robot with a constructor from the easygopigo3 module that was imported as "go".
robot = go.EasyGoPiGo3()

# Set speed for the GoPiGo robot in degrees per second
robot.set_speed(60)

try:
    # Initialize serial connection
    ser = serial.Serial('/dev/ttyUSB0', 115200)
except Exception as e:
    print("Failed to establish a serial connection:")
    print(e)
    sys.exit(-1)

# Make sure arduino is ready to send the data.
print("Syncing serial...0%\r", end='')
while ser.in_waiting == 0:
    ser.write("R".encode())
print("Syncing serial...50%\r", end='')
while ser.in_waiting > 0:
    ser.readline()
print("Syncing serial...100%")
i=0
nahtud = False
while True:

    # Get data from sensors
    arduino_data = getDataFromArduino(ser)

    try:
        # Extract the sensor values
        ls1 = arduino_data['ls1']
        ls2 = arduino_data['ls2']
        ls3 = arduino_data['ls3']
        ls4 = arduino_data['ls4']
        ls5 = arduino_data['ls5']
        near = arduino_data['near']
        clp = arduino_data['clp']
        dist = arduino_data['us1']

        print(ls1,ls2,ls3,ls4,ls5, "DIST: ", dist , "I =" + str(i))

        ############################################################
        # Put your line following and marker detection logic here  #
        ############################################################
        #                                                          #
        #
        
        if clp == 1:                                               #
            robot.backward()                                      #
        elif near == 1:                                            #
            robot.backward()                                      #
        else:                                                      #
            robot.stop()                                           #
        #                                                          #
        #                                                          #
        ############################################################

    except Exception as e:  # Something went wrong
        dist = -1           # Handle the situation.
        print(e)
        pass

    # Throttle the loop to 50 times per second
    #valge on 1
    if i == 7:
        myRobot.stop()
        break
    if ls1 == 0:
        nahtud = True
    if ls1 == 1 and nahtud == True:
        nahtud = False
        i +=1
    if ls2 == 0 and ls3 == 0 and ls4 == 0:
        myRobot.set_motor_dps(myRobot.MOTOR_LEFT, 30)
        myRobot.set_motor_dps(myRobot.MOTOR_RIGHT, 30)
    elif ls2 == 0 and ls3 == 0 and ls4 ==1:
        myRobot.set_motor_dps(myRobot.MOTOR_RIGHT, 30)
        myRobot.set_motor_dps(myRobot.MOTOR_LEFT, 50)
    elif ls2== 0 and ls3==1 and ls4==1:
        myRobot.set_motor_dps(myRobot.MOTOR_RIGHT, 30)
        myRobot.set_motor_dps(myRobot.MOTOR_LEFT, 50)
    elif ls4 == 0 and ls3 == 0 and ls2 ==1:
        myRobot.set_motor_dps(myRobot.MOTOR_RIGHT, 50)
        myRobot.set_motor_dps(myRobot.MOTOR_LEFT, 30)
    elif ls3 == 1 and ls2 == 1 and ls4 ==0:
        myRobot.set_motor_dps(myRobot.MOTOR_RIGHT, 30)
        myRobot.set_motor_dps(myRobot.MOTOR_LEFT, 50)
    elif ls1 == 0 and ls2 == 1 and ls3 ==0 and ls4 ==1 and ls5==1 and dist <<50:
        myRobot.stop()
    else:
        myRobot.set_motor_dps(myRobot.MOTOR_RIGHT, 30)
        myRobot.set_motor_dps(myRobot.MOTOR_LEFT, 30)
        

        
        