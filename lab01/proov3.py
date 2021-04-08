import easygopigo3 as go
import time
myRobot = go.EasyGoPiGo3()
myRobot.set_speed(200)
myRobot.left()
time.sleep(3)
myRobot.forward()
time.sleep(3)
myRobot.right()
time.sleep(1.7)
myRobot.forward()
time.sleep(1.5)
myrobot.right()
time.sleep(0.4)
