import easygopigo3 as go
import time
myRobot = go.EasyGoPiGo3()
myRobot.set_speed(100)
sisend = int(input("Mis on ruudu pikkuseks (sekundites)"))
for i in range (4):
    myRobot.forward()
    time.sleep(sisend)
    myRobot.right()
    time.sleep(3.32)
myRobot.stop()

