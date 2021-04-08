#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Import the simple GoPiGo3 module
import easygopigo3 as go
import time
import signal
import sys
import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import json
import cv2
import _thread
video_capture_device = cv2.VideoCapture(0)

myRobot = go.EasyGoPiGo3()
# Retrieve data from ultrasonic sensor and line following module connected to Arduino

def getDataFromArduino(ser):
    data = []
    ser.write("R".encode()) # Request data

    # Read and parse the serial buffer
    while ser.in_waiting > 0:
        serial_line = ser.readline().strip()
        try:
            data = json.loads(serial_line.decode())
        except Exception as e:
            print(e)
    return data

i = 0
nahtud = False
alustatud = False
varasem = 0
suurused = []

def fastWorker():
    global running, ser, arduino_data, us_pos, enc_pos, cam_pos
    print("Starting fastWorker in a separate thread")
    global i
    global nahtud
    global alustatud
    global varasem
    #Create an instance of the robot with a constructor from the easygopigo3 module that was imported as "go".
    robot = go.EasyGoPiGo3()

    # Set speed for the GoPiGo robot in degrees per second
    robot.set_speed(60)
    

    # Distance from the START marker to the wall in mm
    start_to_wall_dist = 1300 

    #Initialize serial
    ser = serial.Serial(port='/dev/ttyUSB0', baudrate=115200)

    while running:
        # Get the readings of ultrasonic and line following sensor
        # and store them to arduino_data global variable
        arduino_data = getDataFromArduino(ser) 
                    

        ##########################################################
        if alustatud == True:
            kaugus = (myRobot.read_encoders_average()-varasem)*10
            enc_pos = start_to_wall_dist-kaugus   
        ##########################################################
        # enc_pos = ...                                          #
        ##########################################################

        if arduino_data:
            ls1 = arduino_data['ls1']
            ls2 = arduino_data['ls2']
            ls3 = arduino_data['ls3']
            ls4 = arduino_data['ls4']
            ls5 = arduino_data['ls5']
            clp = arduino_data['clp']
            near = arduino_data['near']
            us_pos = arduino_data['us1']

            ############################################################
            # Copy your line following and marker detection logic here #
            ############################################################
            #
            if ls1 == 0:
                nahtud = True
                alustatud = True
            if i == 7:
                myRobot.stop()
                break
            

            if ls1 == 1 and nahtud == True:
                nahtud = False    
                i +=1
                print(i)
            if i == 0:
                varasem = myRobot.read_encoders_average()
            #myRobot.set_speed(60)

                #myRobot.set_motor_dps(myRobot.MOTOR_LEFT, 30)
                #myRobot.set_motor_dps(myRobot.MOTOR_RIGHT, 30)

                pass                
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

            else:
                myRobot.set_motor_dps(myRobot.MOTOR_RIGHT, 30)
                myRobot.set_motor_dps(myRobot.MOTOR_LEFT, 30)
                
                                                         #
                                         #
            #                                                          #
            #                                                          #
            ############################################################

        time.sleep(0.02) # Limit control thread to 50 Hz

    # Stop the robot when not running any more
    print("STOPPING fastWorker")
    robot.stop()
    sys.exit(0)

# Draws a position from the ultrasonic sensor to the map.
def drawUS(pos):
    global curve_us
    x, y = curve_us.getData()
    x = np.append(x, pos)
    y = np.append(y, 700)
    curve_us.setData(x, y)

# Draws a position from encoders to the map.
def drawEnc(pos):
    global curve_enc 
    x, y = curve_enc.getData()
    x = np.append(x, pos)
    y = np.append(y, 500)
    curve_enc.setData(x, y)

# Draws a position from a camera to the map.
def drawCam(pos):
    global curve_cam
    x, y = curve_cam.getData()
    x = np.append(x, pos)
    y = np.append(y, 300)
    curve_cam.setData(x, y)

# Detects the green blob on the wall and returns its diameter in pixels
def getGreenBlobsize():
    Ret, frame = video_capture_device.read()
    frame = cv2.resize(frame, (0,0), fx=0.5, fy=0.5)
    suurused = []
    ####################################################
    # Task 5.1: Implement green blob detection here    #
    ####################################################                         #
    
    lB = 11
    lG = 131
    lR = 121
    hB = 80
    hG = 255
    hR = 219
    
    blobparams = cv2.SimpleBlobDetector_Params()
    blobparams.filterByArea = True
    blobparams.filterByCircularity = False
    blobparams.minDistBetweenBlobs = 800
    blobparams.filterByInertia = False
    blobparams.filterByConvexity = False
    blobparams.minArea = 300
    blobparams.maxArea = 10000000
    
    
    detector = cv2.SimpleBlobDetector_create(blobparams)
    lowerLimits = np.array([lB, lG, lR])
    upperLimits = np.array([hB, hG, hR])
    thresholded = cv2.inRange(frame, lowerLimits, upperLimits)

    outimage = cv2.bitwise_and(frame, frame, mask = thresholded)
    thresholded = cv2.bitwise_not(thresholded)
    keypoints = detector.detect(thresholded)
    for keypoint in keypoints:
        suurused.append(keypoint.size)
    if suurused == []:
        suurused = [0]
    max_size = max(suurused)
    
    
    
    return max_size



#
    # ......                                           #
    #################################################### 


def getDistanceWithCam(blob_size):
    a = 66578.23
    b = -163.64
    distance = a * 1/blob_size + b
    print("Blob : " + str(blob_size))
    print("Dist : " + str(distance))
    if blob_size > 0:
        cam_pos = distance

        ######################################################
        # Task 5.2: Calculate distance based on the bob size #
        ######################################################
        # distance = ...                                     #
        ######################################################

    return distance

# Slower code goes here
def slowWorker():
    global us_pos, enc_pos, cam_pos

    # Get the blob size and convert it to distance from the wall
    blob_size = getGreenBlobsize()
    cam_pos = getDistanceWithCam(blob_size)

    # Update the graphs only when the values are valid
    if (us_pos >= 0):
        drawUS(us_pos)

    if (enc_pos >= 0):
        drawEnc(enc_pos)

    if (cam_pos >= 0):
        drawCam(cam_pos)

# This function will be called when CTRL+C is pressed
def signal_handler(sig, frame):
    print('\nYou pressed Ctrl+C! Closing the program nicely :)')
    global running
    running = False
    try:
        ser.close()
    except Exception:
        pass
    sys.exit(0)

# Register a callback for CTRL+C
signal.signal(signal.SIGINT, signal_handler)

# Create a window for plotting
win = pg.GraphicsWindow()
win.setWindowTitle('Plotter')
win.resize(1024,460)
plot1 = win.addPlot()
plot1.setLabel('top', "Distance (mm)")
plot1.setXRange(-200,2000)
plot1.setYRange(0,5000)
plot1.addLegend()

# Initialize curves for each sensor
curve_us = plot1.plot([], [], pen=pg.mkPen(width=5, color='r'), brush=pg.mkBrush(radius=10, color='r'), symbol='o', symbolBrush='r', symbolSize=15, name='Ultrasonic')
curve_enc = plot1.plot([], [], pen=pg.mkPen(width=5, color='g'), symbol='o', symbolBrush='g', symbolSize=15, name='Encoders')
curve_cam = plot1.plot([], [], pen=pg.mkPen(width=5, color='b'), symbol='o', symbolBrush='b', symbolSize=15, name='Camera')

# Load a background image of a track
img_arr = np.asarray(cv2.cvtColor(cv2.imread('map.png'),cv2.COLOR_BGR2RGB))
img_item = pg.ImageItem(np.rot90(img_arr, -1))
img_item.scale(1.1, 10)
img_item.setZValue(-100)
plot1.addItem(img_item)

# Create global variables for the latest positions
us_pos = -1
enc_pos = -1
cam_pos = -1

# open the camera
cap = cv2.VideoCapture(0)

running = True    # A state variable for keeping the fastWorker thread running
arduino_data = [] # This will be filled with decoded JSON data from Arduino

_thread.start_new_thread(fastWorker, ()) # Start fastWorker in a separate thread.

# Create timer and connect it to slowWorker.
# Effectively, slowWorker() will be called no more than 10 times per second.
timer = pg.QtCore.QTimer()
timer.timeout.connect(slowWorker)
timer.start(100)

# Execute the Qt application. This function is blocking until the user closes the window.
if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
    QtGui.QApplication.instance().exec_()

# The window has been closed. Stop whatever we were doing.
running = False # Stop fastWorker
timer.stop()    # Stop slowWorker
ser.close()     # Disconnect from Arduino
