#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Import the simple GoPiGo3 module
import easygopigo3 as go
# Import time
import time

myRobot = go.EasyGoPiGo3()
myRobot.set_speed(600)
for i in range (5):
    myRobot.forward()
    time.sleep(1)
    myRobot.backward()
    time.sleep(1)
myRobot.stop()

