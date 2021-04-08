#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import easygopigo3 as go
import numpy as np
import time


# global variable for determining GoPiGo speed
myRobot = go.EasyGoPiGo3()
# global variable for video feed
cap = cv2.VideoCapture(0)

kernel2=7





#robot object
myRobot = go.EasyGoPiGo3()


def updateValue(new_value):
    global lB
    lB = new_value
    return

def updateValue1(new_value):
    global lG
    lG = new_value
    return

def updateValue2(new_value):
    global lR
    lR = new_value
    return

def updateValue3(new_value):
    global hB
    hB = new_value
    return

def updateValue4(new_value):
    global hG
    hG = new_value
    return

def updateValue5(new_value):
    global hR
    hR = new_value
    return

def kernel(new_value):
    global kernel2
    kernel2 = new_value
    return






# TASK 1
def get_line_location(frame):
    lowerLimits = np.array([lB, lG, lR])
    upperLimits = np.array([hB, hG, hR])
    thresholded = cv2.inRange(frame, lowerLimits, upperLimits)
    kernel = np.ones((kernel2,kernel2),np.uint8)
    thresholded = cv2.erode(thresholded,kernel,iterations = 1)
    linelocation =np.nonzero(thresholded)
    cv2.imshow('Blob',thresholded )
    linelocation = np.mean(linelocation[1])
    
    return linelocation


# TASK 2
def bang_bang(linelocation):
    if linelocation < 320:
        myRobot.left()
    else:
        myRobot.right()


    
    return


# TASK 3
def bang_bang_improved(linelocation):
    if linelocation < 270:
        myRobot.left()
    elif linelocation > 370:
        myRobot.right()
    else:
        myRobot.forward()
    
    return


# TASK 4
def proportional_controller(linelocation):
    q = linelocation*0.9
    print(linelocation)
    myRobot.set_speed(450)
    try:
        linelocation = int(linelocation)
    except:
        myRobot.stop()
    if linelocation < 270:
        myRobot.set_motor_dps(myRobot.MOTOR_LEFT, 450+q)
        myRobot.set_motor_dps(myRobot.MOTOR_RIGHT, 450-q)
    if linelocation > 370:
        myRobot.set_motor_dps(myRobot.MOTOR_LEFT, 450-q)
        myRobot.set_motor_dps(myRobot.MOTOR_RIGHT, 450+q)

    return

integral = 0
prevError = 0
# TASK 5
def pid_controller(linelocation):
    global integral
    global prevError
    try:
        linelocation = int(linelocation)
        
        error = 320 - linelocation
        Tu = 80
        Ku = 0.8
        Kp = 0.4
        
    
        Ki = (1.2*Ku)/Tu
        Kd = (3*Ku*Tu)/40
        prop = Kp*error
        integral = integral + Ki*error
    
        derivative = Kd * (error - prevError)
        kiirus = 450
        q = prop+integral+derivative

        myRobot.set_speed(450)

        prevError = error
         

        if linelocation > 1:
            myRobot.set_motor_dps(myRobot.MOTOR_LEFT, kiirus-q)
            myRobot.set_motor_dps(myRobot.MOTOR_RIGHT, kiirus+q)

    except:
        myRobot.stop()
 
   
    

                       
    return


def init():
    
    try:
        f = open("trackbar_defaults.txt")
        values = []
        for i in f:
            values.append(i)
        f.close()
    except IOError:
        values = [0,0,0,255,255,250]
    
    
    
    updateValue(int(values[0]))
    updateValue1(int(values[1]))
    updateValue2(int(values[2]))
    updateValue3(int(values[3]))
    updateValue4(int(values[4]))
    updateValue5(int(values[5]))
    
    cv2.namedWindow('Aken')
    cv2.createTrackbar("LowerB", "Aken", lB, 255, updateValue)
    cv2.createTrackbar("LowerG", "Aken", lG, 255, updateValue1)
    cv2.createTrackbar("LowerR", "Aken", lR, 255, updateValue2)
    cv2.createTrackbar("upperB", "Aken", hB, 255, updateValue3)
    cv2.createTrackbar("upperG", "Aken", hG, 255, updateValue4)
    cv2.createTrackbar("upperR", "Aken", hR, 255, updateValue5)
    cv2.createTrackbar("Kerneli suurus", "Aken", kernel2, 50, kernel)

    
    
    return

# Initialization
init()

while True:
    ret, frame = cap.read() #cap on videofeed
    frame = frame[250:300]
    linelocation = get_line_location(frame)
    
    
    
    
    # Task 2: uncomment the following line and implement bang_bang function.
    #bang_bang(linelocation)
    
    # Task 3: uncomment the following line and implement bang_bang_improved function.
    #bang_bang_improved(linelocation)
    
    # Task 4: uncomment the following line and implement proportional_controller function.
    #proportional_controller(linelocation)
  
    # Task 5: uncomment the following line and implement pid_controller function.
    pid_controller(linelocation)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        f = open("trackbar_defaults.txt","w+")
        values = [lB,lG,lR,hB,hG,hR]
        for i in values:
            f.write(str(i))
            f.write("\n")
        f.close()
        break
        

cap.release()
cv2.destroyAllWindows()
myRobot.stop()
