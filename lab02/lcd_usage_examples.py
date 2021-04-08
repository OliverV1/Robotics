#!/bin/python3
# -*- coding: utf-8 -*-

# THIS CODE ISN'T MEANT TO BE RUN!
# It is meant to help you understand how the LCD library works

# First you need to import the LCD API that you just downloaded.
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
# Now you need to initialize the LCD and get an instance of it.
lcd_columns = 16
lcd_rows = 2
i2c = busio.I2C(board.SCL, board.SDA)
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
# The color of the display. Provide a list of three integers ranging 0 - 100, 
# [R, G, B]. 0 is no color, or “off”. 100 is maximum color.
lcd.color = [R, G, B]

# This function clears all text on the display and
# resets the cursor position to the first character
lcd.clear()
# Prints the text on the LCD. The ‘\n’ symbol in the string
# changes lines at that position.
lcd.message = 'text here \n This is at a new line now'
# This variable contains a boolean value (True/False). 
# Each button has its own name, you have to replace 'left_button' correspondingly.
lcd.left_button

# To run Linux commands from Python(to get CPU temperature and IP) import Popen and PIPE.
from subprocess import Popen, PIPE
# To get current time import the time module
import datetime

# You can use the following function to read the CPU temperature.
# The function runs the Linux command 'vcgencmd -measure_temp'.
def getCPUtemperature():
    p = Popen(["vcgencmd", "measure_temp"], stdin=PIPE, stdout=PIPE, stderr=PIPE)
    output, err = p.communicate()
    return (output.decode().replace("temp=", "").replace("'C\n", ""))

# The following function can be used to get the current time
# Return current time as a character string
def getTimeText():
    t = datetime.datetime.now()
    hours = str(t.hour)
    minutes = str(t.minute)
    seconds = str(t.second)
    timetext = hours + ':' + minutes + ':' + seconds
    return timetext

# The following function can be used to get the Raspberry's IP address
# Return device IP as a character string
def getIP():
    p = Popen(["hostname", "-I"], stdin=PIPE, stdout=PIPE, stderr=PIPE)
    output, err = p.communicate()
    return output.decode()
