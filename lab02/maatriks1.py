#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time

# reference pins by GPIO numbers
GPIO.setmode(GPIO.BCM)
# disable warnings
GPIO.setwarnings(False)

# define row and column pin numbers
row_pins = [21, 20, 16, 19, 13, 6, 5]
col_pins = [2, 3, 4, 14, 15]

# set all the pins as outputs and set column pins high, row pins low
GPIO.setup(col_pins, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(row_pins, GPIO.OUT, initial=GPIO.LOW)

# Sets the waiting time between rows. With larger wait times (0.1) you can see that rows are lit up at different times. With smaller times (0.01) the LEDs appear to be not blinking at all
wait_time = 0.01

def showRow(rowNumber, columns, delay):
    GPIO.output(row_pins[rowNumber], GPIO.HIGH)
    for i in columns:
        GPIO.output(col_pins[i], GPIO.LOW)
        time.sleep(delay)
        GPIO.output(col_pins[i], GPIO.HIGH)
    
                    
    GPIO.output(row_pins[rowNumber], GPIO.LOW)
    

# Displays image 50 times
for i in range(250):

    showRow (2, [1,3,4] ,0.001)

for i in range(250):
    
    showRow(0,[2],0.001)
    showRow(1,[1,2,3],0.001)
    showRow(2,[0,2,4],0.001)
    showRow(3,[2],0.001)
    showRow(4,[2],0.001)
    showRow(5,[2],0.001)
    showRow(6,[2],0.001)
    
for i in range(250):
    showRow(1,[2],0.001)
    showRow(2,[3],0.001)
    showRow(3,[0,1,2,3,4],0.001)
    showRow(4,[3],0.001)
    showRow(5,[2],0.001)
    
for i in range(250):
    showRow(1,[1,2],0.001)
    showRow(0,[3],0.001)
    showRow(2,[4],0.001)
    showRow(3,[2,3],0.001)
    showRow(5,[2],0.001)


    
    


# reset GPIO
GPIO.cleanup()

