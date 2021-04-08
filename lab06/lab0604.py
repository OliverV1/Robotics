import numpy as np
import cv2
import time

kernel2=4
import easygopigo3 as go
myRobot = go.EasyGoPiGo3()
myRobot.set_speed(20)
punktid_leitud = False

values = []
try:
    f = open("trackbar_defaults.txt")
    for i in f:
        values.append(i)
    f.close()
except IOError:
    values = [0,0,0,255,255,250]


# Open the camera


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

updateValue(int(values[0]))
updateValue1(int(values[1]))
updateValue2(int(values[2]))
updateValue3(int(values[3]))
updateValue4(int(values[4]))
updateValue5(int(values[5]))

blobparams = cv2.SimpleBlobDetector_Params()
blobparams.filterByArea = True
blobparams.filterByCircularity = False
blobparams.minDistBetweenBlobs = 1
blobparams.filterByInertia = False
blobparams.filterByConvexity = False
blobparams.minArea = 100
blobparams.maxArea = 100000000
detector = cv2.SimpleBlobDetector_create(blobparams)


cv2.namedWindow('Aken')
cv2.createTrackbar("Kerneli suurus", "Aken", kernel2, 50, kernel)
cv2.createTrackbar("LowerB", "Aken", lB, 255, updateValue)
cv2.createTrackbar("LowerG", "Aken", lG, 255, updateValue1)
cv2.createTrackbar("LowerR", "Aken", lR, 255, updateValue2)
cv2.createTrackbar("upperB", "Aken", hB, 255, updateValue3)
cv2.createTrackbar("upperG", "Aken", hG, 255, updateValue4)
cv2.createTrackbar("upperR", "Aken", hR, 255, updateValue5)

video_capture_device = cv2.VideoCapture(0)
while True:
    Ret, frame = video_capture_device.read()
    frame = frame[200:300]
    lowerLimits = np.array([lB, lG, lR])
    upperLimits = np.array([hB, hG, hR])
    thresholded = cv2.inRange(frame, lowerLimits, upperLimits)

    outimage = cv2.bitwise_and(frame, frame, mask = thresholded)
    
    kernel = np.ones((kernel2,kernel2),np.uint8)
    
    thresholded = cv2.erode(thresholded,kernel,iterations = 1)
    thresholded = cv2.bitwise_not(thresholded)
    thresholded = cv2.copyMakeBorder(
             thresholded, 
             5, 
             5, 
             5, 
             5, 
             cv2.BORDER_CONSTANT, 
             value=255
          )

    keypoints = detector.detect(thresholded)
    obama = cv2.drawKeypoints(thresholded, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    
   
    punktid_leitud = False
    olemasolu = False
    cv2.imshow('frame', outimage)
    cv2.imshow('Nonnii', obama)

       
    if len(keypoints) == 2:
        pööre = (int(keypoints[1].pt[0]) + int(keypoints[0].pt[0]))
        pööre = pööre/2
        print(pööre)
        punktid_leitud = True
        if pööre < 100:
            myRobot.spin_right()
            myRobot.forward()
 
        if pööre > 140:
            myRobot.spin_left()
            myRobot.forward()
       
        if int(keypoints[1].pt[0]) < 20 and int(keypoints[0].pt[0]) > 620:
            olemasolu = True
    else:
        myRobot.spin_right()
        
    if olemasolu == True:
        if punktid_leitud == True:
            myRobot.set_speed(1000)
            myRobot.forward()
            time.sleep(3)
            punktid_leitud = False
            olemasolu = False
            myRobot.stop()
            break


        
    
        
        

        
  
    













    if cv2.waitKey(1) & 0xFF == ord('q'):
        myRobot.stop()
        break



f = open("trackbar_defaults.txt","w+")
values = [lB,lG,lR,hB,hG,hR]
for i in values:
    f.write(str(i))
    f.write("\n")
f.close()

    


# When everything done, release the capture
print('closing program')
cv2.destroyAllWindows()

