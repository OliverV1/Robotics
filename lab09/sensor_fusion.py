#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Import the simple GoPiGo3 module and other modules needed for the lab
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
import scipy.stats as stats

# Update this variable every time you move to the next task, keep in mind that this is a number not a string
CURRENT_LAB09_TASK = 4.4
myRobot = go.EasyGoPiGo3()
video_capture_device = cv2.VideoCapture(0)
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
keskminelist = []
keskminesamm = 0
eelmine_pos0 = 0
eelmine_pos1 = 0
eelmine_pos2 = 0
eelmine_pos3 = 0
eelmine_pos4 = 0
eelmine_pos5 = 0
y0 = 0
y1 = 0
y2 = 0
y3 = 0
y4 = 0
y5 = 0
eelmine_pos = 0
kaugused_list= 0
eelmine_enc_pos = 0
mu1 = 0
sigma1=0
mu2=0
sigma2=0

positsioon_eelmine = 0
enc_eelmine = 0
hedhog_eelmine = 0 #ragnari firma







def fastWorker():
    global running, ser, arduino_data, us_pos, enc_pos, cam_pos
    print("Starting fastWorker in a separate thread")

    #Create an instance of the robot with a constructor from the easygopigo3 module that was imported as "go".
    robot = go.EasyGoPiGo3()

    # Set speed for the GoPiGo robot in degrees per second
    robot.set_speed(60)
    robot.reset_encoders()

    # Distance from the START marker to the wall in mm
    start_to_wall_dist = 1600

    # Initialize serial
    ser = serial.Serial(port='/dev/ttyUSB0', baudrate=115200)

    while running:
        # Get the readings of ultrasonic and line following sensor
        # and store them to arduino_data global variable
        arduino_data = getDataFromArduino(ser)
        print(arduino_data)

        ##########################################################
        # Task 4: Get the averaged encoder value and use it to   #
        #         find the distance from the wall in millimetres #
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
            global i
            global nahtud############################################################
            enc_pos = 1600 - myRobot.read_encoders_average()*10

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
            #                                                          #
            #                                                          #
            #                                                          #
            ############################################################

        else:
            print("No data received from Arduino!")

        time.sleep(0.02) # Limit control thread to 50 Hz

    # Stop the robot when not running any more
    print("STOPPING fastWorker")
    robot.stop()
    sys.exit(0)

# Moving average filter
def movingAverage(pos):
    global keskminelist
    if pos > 0:
        keskminelist.append(pos)
        keskmine = sum(keskminelist)/len(keskminelist)
        if len(keskminelist)>20:
            del keskminelist[0]
     
        return keskmine
    else:
        pass

# Complementary filter
def complementary(us_pos, enc_pos):
    global positsioon_eelmine
    global enc_eelmine
    global hedhog_eelmine
    
    kaamera_kaal = 0.9
    hedhog_kaal = 0.1
    positsioon = hedhog_kaal*us_pos+(kaamera_kaal*(positsioon_eelmine+(enc_pos-enc_eelmine)))
    positsioon_eelmine = positsioon
    enc_eelmine = enc_pos
    hedhog_eelmine = us_pos
    return positsioon

# Draws a position from the ultrasonic sensor to the map.
def drawUS(pos):
    global curve_us
    x, y = curve_us.getData()
    x = np.append(x, pos)
    y = np.append(y, 2200)
    curve_us.setData(x, y)

# Draws a moving average of the ultrasonic sensor position to the map.
def drawMovingAverageUS(pos):
    global curve_ma_us
    x, y = curve_ma_us.getData()
    x = np.append(x, pos)
    y = np.append(y, 1800)
    curve_ma_us.setData(x, y)

# Draws a position from encoders to the map.
def drawEnc(pos):
    global curve_enc
    print(pos)
    x, y = curve_enc.getData()
    x = np.append(x, pos)
    y = np.append(y, 1400)
    curve_enc.setData(x, y)

# Draws a complementary filtered result to the map.
def drawCompl(pos):
    global curve_compl
    x, y = curve_compl.getData()
    x = np.append(x, pos)
    y = np.append(y, 1000)
    curve_compl.setData(x, y)

# Draws a position from a camera to the map.
def drawCam(pos):
    global curve_cam
    x, y = curve_cam.getData()
    x = np.append(x, pos)
    y = np.append(y, 600)
    curve_cam.setData(x, y)

# Draws a position from Kalman filter to the map.
def drawKalman(pos):
    global curve_kalman
    x, y = curve_kalman.getData()
    x = np.append(x, pos)
    y = np.append(y, 200)
    curve_kalman.setData(x, y)

# Calculates velocity for a given sensor based on given position measurement.
def getVelocity(pos, sensor):
    global y0
    global y1
    global y2
    global y3
    global y4
    global y5
    global eelmine_pos0
    global eelmine_pos1
    global eelmine_pos2
    global eelmine_pos3
    global eelmine_pos4
    global eelmine_pos5
    if sensor == "US":
        pos0 = pos
        x0 = time.time()
        kiirus0 = (pos0 - eelmine_pos0)/(x0-y0)
        y0 = x0
        eelmine_pos0 = pos
        return kiirus0

    if sensor == "Enc":
        pos1 = pos
        x1 = time.time()
        kiirus1 = (pos1 - eelmine_pos1)/(x1-y1)
        y1 = x1
        eelmine_pos1 = pos
        return kiirus1
    if sensor == "Cam":
        pos2 = pos
        x2 = time.time()
        kiirus2 = (pos2 - eelmine_pos2)/(x2-y2)
        eelmine_pos2 = pos
        y2 = x2
        return kiirus2
    if sensor == "US_MA":
        pos3 = pos
        x3= time.time()
        kiirus3 = (pos3 - eelmine_pos3)/(x3-y3)
        eelmine_pos23 = pos
        y3 = x3
        return kiirus3
    if sensor == "Kalman":
        pos4 = pos
        x4 = time.time()
        kiirus4 = (pos4 - eelmine_pos4)/(x4-y4)
        eelmine_pos4 = pos
        y4 = x4
        return kiirus4
    if sensor == "Compl":
        pos5 = pos
        x5 = time.time()
        kiirus5 = (pos5 - eelmine_pos5)/(x5-y5)
        eelmine_pos5 = pos
        y5 = x5
        return kiirus5

  

# Draws the velocity of the robot calculated from US measurements to the plot.
def drawUSVelocity(pos):
    global curve_us_vel, STARTTIME

    x, y = curve_us_vel.getData()
    velocity = getVelocity(pos, "US")
    x = np.append(x[1:], time.time()-STARTTIME)
    y = np.append(y[1:], velocity)
    curve_us_vel.setData(x, y)

# Draws the velocity of the robot calculated from US moving average measurements to the plot.
def drawMAUSVelocity(pos):
    global curve_ma_us_vel, STARTTIME

    x, y = curve_ma_us_vel.getData()
    velocity = getVelocity(pos, "US_MA")
    x = np.append(x[1:], time.time()-STARTTIME)
    y = np.append(y[1:], velocity)
    curve_ma_us_vel.setData(x, y)

# Draws the velocity of the robot calculated from encoder measurements to the plot.
def drawEncVelocity(pos):
    global curve_enc_vel, STARTTIME

    x, y = curve_enc_vel.getData()
    velocity = getVelocity(pos, "Enc")
    x = np.append(x[1:], time.time()-STARTTIME)
    y = np.append(y[1:], velocity)
    curve_enc_vel.setData(x, y)

# Draws the velocity of the robot calculated from complementary filtered results to the plot.
def drawComplVelocity(pos):
    global curve_compl_vel, STARTTIME

    x, y = curve_compl_vel.getData()
    velocity = getVelocity(pos, "Compl")
    x = np.append(x[1:], time.time()-STARTTIME)
    y = np.append(y[1:], velocity)
    curve_compl_vel.setData(x, y)

# Draws the velocity of the robot calculated from camera measurements to the plot.
def drawCamVelocity(pos):
    global curve_cam_vel, STARTTIME

    x, y = curve_cam_vel.getData()
    velocity = getVelocity(pos, "Cam")
    x = np.append(x[1:], time.time()-STARTTIME)
    y = np.append(y[1:], velocity)
    curve_cam_vel.setData(x, y)

# Draws the velocity of the robot calculated from Kalman filtered measurements to the plot.
def drawKalmanVelocity(pos):
    global curve_kalman_vel, STARTTIME

    x, y = curve_kalman_vel.getData()
    velocity = getVelocity(pos, "Kalman")
    x = np.append(x[1:], time.time()-STARTTIME)
    y = np.append(y[1:], velocity)
    curve_kalman_vel.setData(x, y)

# Detects the green blob on the wall and returns its diameter in pixels
def getGreenBlobsize():
    Ret, frame = video_capture_device.read()
    frame = cv2.resize(frame, (0,0), fx=0.5, fy=0.5)
    suurused = []
    max_size = 0
    
    lB = 0
    lG = 122
    lR = 62
    hB = 74
    hG = 255
    hR = 255
    
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

def getDistanceWithCam(blob_size):
    a = 66578.23
    b = -163.64
    if blob_size > 0:
        dist = a * 1/blob_size + b
        
        if 0 < dist < 1700:
            return dist
    
        ######################################################
        # Task 5.2: Calculate distance based on the bob size #
        ######################################################
        # distance = ...                                     #
        ######################################################
        else:
            return -1
    return -1

# The predict step of Kalman filter
def predict(mu1, sigma1, mu2, sigma2):
    ################################################################
    # Lab 09 Task 4.2: Implement the predict step.                 #
    # The code should return a tuple of two values:                #
    # the mean and standard deviation of the resulting Gaussian.   #
    ################################################################
    mu = mu1 + mu2
    sigma = (sigma1**2+sigma2**2)**0.5
    return (mu, sigma) # Change this

# The update step of Kalman filter
def update(mu1, sigma1, mu2, sigma2):
    ################################################################
    # Lab 09 Task 4.3: Implement the update step.                  #
    # The code should return a tuple of two values:                #
    # the mean and standard deviation of the resulting Gaussian.   #
    ################################################################

    
    mu_res = ((mu1*sigma2**2)+(mu2*sigma1**2))/(sigma1**2+sigma2**2)
    sigma_res = (sigma1**-2+sigma2**-2)**-0.5
    return (mu_res, sigma_res) # Change this

# The Kalman filter
def kalman(cam_pos, enc_pos):
    global current_location_mu, current_location_sigma
    global camera_mu,eelmine_enc_pos,eelmine_pos, camera_sigma, encoder_mu, encoder_sigma

    ################################################################
    # Lab 09 Task 4.1: Update Gaussians for camera and encoders.   #
    ################################################################
    # Correct the following values.
    if cam_pos > 0 and cam_pos <1700:
        camera_mu = cam_pos
    camera_sigma = 44
    encoder_mu = enc_pos-eelmine_enc_pos
    encoder_sigma = 10
    eelmine_enc_pos = enc_pos
    ################################################################

    if CURRENT_LAB09_TASK == 4.2:
        #################################################################
        # Lab 09 Task 4.2                                               #
        # Initialize the current_location_mu and current_location_sigma #
        # parameters using a measurement from camera.                   #
        # Then update it on every encoder measurement.                  #
        #################################################################
        if current_location_mu == None or current_location_sigma == None:
            current_location_mu = camera_mu
            current_location_sigma = camera_sigma
        else:
            current_location_mu, current_location_sigma = predict(current_location_mu,current_location_sigma,encoder_mu,encoder_sigma)
        

    elif CURRENT_LAB09_TASK == 4.3:
        #################################################################
        # Lab 09 Task 4.3                                               #
        # Initialize the current_location_mu and current_location_sigma #
        # parameters using a measurement from camera.                   #
        # Then update it on every new camera measurement.               #
        # Also stop the robot from moving                               #
        # by modifying your line following code.                        #
        #################################################################
     
        if current_location_mu == None or current_location_sigma == None:
            current_location_mu = camera_mu
            current_location_sigma = camera_sigma
        else:
            current_location_mu, current_location_sigma = update(current_location_mu,current_location_sigma,camera_mu,camera_sigma)
        
        

    elif CURRENT_LAB09_TASK == 4.4:
        if current_location_mu == None or current_location_sigma == None:
            current_location_mu = camera_mu
            current_location_sigma = camera_sigma
     
        current_location_mu, current_location_sigma = predict(current_location_mu,current_location_sigma,encoder_mu,encoder_sigma)
        if cam_pos != -1:
            print("Mu" + str(current_location_mu))
            print("Sigma" + str(current_location_sigma))
            current_location_mu, current_location_sigma = update(current_location_mu,current_location_sigma,camera_mu,camera_sigma)


# Slower code goes here
def slowWorker():
    global us_pos, enc_pos, cam_pos, plot2
    global current_location_mu, current_location_sigma
    global camera_mu, camera_sigma, encoder_mu, encoder_sigma

    # Get the blob size and convert it to distance from the wall
    blob_size = getGreenBlobsize()
    cam_pos = getDistanceWithCam(blob_size)

    # Adjust the x-coordinate range for plot2
    if CURRENT_LAB09_TASK >= 1:
        plot2.setXRange(time.time()-STARTTIME-5,time.time()-STARTTIME)

    # Update the graphs only when the values are valid
    if us_pos != None and CURRENT_LAB09_TASK < 4:
        drawUS(us_pos)
        if CURRENT_LAB09_TASK >= 1:
            drawUSVelocity(us_pos)
        if CURRENT_LAB09_TASK == 2:
            ma_pos = movingAverage(us_pos)
            drawMovingAverageUS(ma_pos)
            drawMAUSVelocity(ma_pos)

    if us_pos != None and enc_pos != None and CURRENT_LAB09_TASK == 3:
        compl_pos = complementary(us_pos, enc_pos)
        drawCompl(compl_pos)
        drawComplVelocity(compl_pos)

    if enc_pos != None and CURRENT_LAB09_TASK != 2:
        drawEnc(enc_pos)
        if CURRENT_LAB09_TASK >= 1:
            drawEncVelocity(enc_pos)

    if cam_pos != None and CURRENT_LAB09_TASK <= 1 or CURRENT_LAB09_TASK >= 4:
        drawCam(cam_pos)
        if CURRENT_LAB09_TASK >= 1:
            drawCamVelocity(cam_pos)

    if CURRENT_LAB09_TASK >= 4:
        kalman(cam_pos, enc_pos)

        if camera_mu != None:
            plot_gaussian(camera_mu, camera_sigma, 0)
        if encoder_mu != None:
            plot_gaussian(encoder_mu, encoder_sigma, 1)

        if current_location_mu != None:
            plot_gaussian(current_location_mu, current_location_sigma, 2)
            if CURRENT_LAB09_TASK == 4.4:
                drawKalman(current_location_mu)
                drawKalmanVelocity(current_location_mu)

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

# Plots Gaussian with given mu, sigma
def plot_gaussian(mu, sigma, gaussian_index):
    global gaussians
    x = np.arange(-200,2000)
    y = stats.norm.pdf(x, mu, sigma)
    gaussians[gaussian_index].setData(x, y)


# Register a callback for CTRL+C
signal.signal(signal.SIGINT, signal_handler)

# Create a window for plotting
win = pg.GraphicsWindow()
win.setWindowTitle('Plotter')
if CURRENT_LAB09_TASK < 1:
    win.resize(1024,480)
else:
    win.resize(1024,900)
plot1 = win.addPlot()
plot1.setLabel('top', "Distance (mm)")
plot1.setXRange(-200,2000)
plot1.setYRange(0,5000)
plot1.addLegend()
plot2 = None
if CURRENT_LAB09_TASK >= 4:
    win.nextRow()
    plot3 = win.addPlot()
    plot3.setXRange(-200,2000)
    plot3.addLegend()
if CURRENT_LAB09_TASK >= 1:
    win.nextRow()
    plot2 = win.addPlot()
    plot2.setLabel('top', "Velocity (mm/s)")
    plot2.addLegend()

# Initialize curves for each sensor
STARTTIME = time.time()
VEL_INIT_X = list(reversed([ -x*0.1 for x in range(50) ]))
VEL_INIT_Y = [0]*50
if CURRENT_LAB09_TASK < 4:
    curve_us = plot1.plot([], [], pen=pg.mkPen(width=5, color='r'), symbol='o', symbolBrush='r', symbolSize=15, name='Ultrasonic')
    if CURRENT_LAB09_TASK >= 1:
        curve_us_vel = plot2.plot(VEL_INIT_X[:], VEL_INIT_Y[:], pen=pg.mkPen(width=1, color='r'), name='Ultrasonic')
if CURRENT_LAB09_TASK <= 1 or CURRENT_LAB09_TASK >= 3:
    curve_enc = plot1.plot([], [], pen=pg.mkPen(width=5, color='g'), symbol='o', symbolBrush='g', symbolSize=15, name='Encoders')
    if CURRENT_LAB09_TASK >= 1:
        curve_enc_vel = plot2.plot(VEL_INIT_X[:], VEL_INIT_Y[:], pen=pg.mkPen(width=1, color='g'), name='Encoders')
if CURRENT_LAB09_TASK <= 1 or CURRENT_LAB09_TASK >= 4:
    curve_cam = plot1.plot([], [], pen=pg.mkPen(width=5, color='b'), symbol='o', symbolBrush='b', symbolSize=15, name='Camera')
    if CURRENT_LAB09_TASK >= 1:
        curve_cam_vel = plot2.plot(VEL_INIT_X[:], VEL_INIT_Y[:], pen=pg.mkPen(width=1, color='b'), name='Camera')
if CURRENT_LAB09_TASK == 2:
    curve_ma_us = plot1.plot([], [], pen=pg.mkPen(width=5, color=1), symbol='o', symbolBrush=1, symbolSize=15, name='Ultrasonic MA')
    curve_ma_us_vel = plot2.plot(VEL_INIT_X[:], VEL_INIT_Y[:], pen=pg.mkPen(width=1, color=1), name='Ultrasonic MA')
if CURRENT_LAB09_TASK == 3:
    curve_compl = plot1.plot([], [], pen=pg.mkPen(width=5, color=2), symbol='o', symbolBrush=2, symbolSize=15, name='Complementary')
    curve_compl_vel = plot2.plot(VEL_INIT_X[:], VEL_INIT_Y[:], pen=pg.mkPen(width=1, color=2), name='Complementary')
if CURRENT_LAB09_TASK >= 4.4:
    curve_kalman = plot1.plot([], [], pen=pg.mkPen(width=5, color='w'), symbol='o', symbolBrush='w', symbolSize=15, name='Kalman')
    curve_kalman_vel = plot2.plot(VEL_INIT_X[:], VEL_INIT_Y[:], pen=pg.mkPen(width=1, color='w'), name='Kalman')

# Initialize curves for Kalman filter
if CURRENT_LAB09_TASK >= 4:
    gaussian_names = ["Camera", "Encoders", "Filtered result"]
    gaussian_colors = ['b', 'g', 'w']
    gaussians = [ plot3.plot([], [], pen=pg.mkPen(width=1, color=gaussian_colors[i]), name=gaussian_names[i]) for i in range(3) ]

# Load a background image of a track
img_arr = np.asarray(cv2.cvtColor(cv2.imread('map.png'),cv2.COLOR_BGR2RGB))
img_item = pg.ImageItem(np.rot90(img_arr, -1))
img_item.scale(1.1, 10)
img_item.setZValue(-100)
plot1.addItem(img_item)

# Create global variables for the latest positions and Kalman filtering
us_pos = None
enc_pos = None
cam_pos = None
current_location_mu = None # This is the mean value for Gaussian representing our current location estimate in tasks 4.2 to 4.4
current_location_sigma = None # This is the standard deviation value for Gaussian representing our current location estimate in tasks 4.2 to 4.4
camera_mu = None
camera_sigma = None
encoder_mu = None
encoder_sigma = None

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
