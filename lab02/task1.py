import board
import busio
import datetime
import time
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
lcd_columns = 16
lcd_rows = 2
i2c = busio.I2C(board.SCL, board.SDA)
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.color = [100, 50, 10]
from subprocess import Popen, PIPE
def getCPUtemperature():
    p = Popen(["vcgencmd", "measure_temp"], stdin=PIPE, stdout=PIPE, stderr=PIPE)
    output, err = p.communicate()
    return (output.decode().replace("temp=", "").replace("'C\n", ""))
def getTimeText():
    t = datetime.datetime.now()
    hours = str(t.hour)
    minutes = str(t.minute)
    seconds = str(t.second)
    timetext = hours + ':' + minutes + ':' + seconds
    return timetext
def getIP():
    p = Popen(["hostname", "-I"], stdin=PIPE, stdout=PIPE, stderr=PIPE)
    output, err = p.communicate()
    return output.decode()
while True:
    if (lcd.right_button):
        lcd.clear()
        lcd.message = ("Time: "  + getTimeText())
    if(lcd.left_button):
        lcd.clear()
        lcd.message = ("CPU: "+ getCPUtemperature())
    if(lcd.up_button):
        lcd.clear()
        lcd.message = ("IP: "+ getIP())
    if(lcd.down_button):
        lcd.clear()
        break
    if(lcd.select_button):
        lcd.clear()
        scroll_message = "on suurtel meestel väiksed teod \n the big men have small snails"
        lcd.message = scroll_message
        time.sleep(2)
        for i in range(len(scroll_message) + 16):
            if (lcd.select_button):
                time.sleep(0.5)
                lcd.clear()
                break
            lcd.move_left()
            time.sleep(0.5)
            
        
    
        



