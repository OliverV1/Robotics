import numpy as np
import cv2
import time
lopp = 0
lopp2 = 0
lopp3=0
kernel2=1

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
blobparams.minDistBetweenBlobs = 800
blobparams.filterByInertia = False
blobparams.filterByConvexity = False
blobparams.minArea = 300
blobparams.maxArea = 1000000
detector = cv2.SimpleBlobDetector_create(blobparams)
cv2.namedWindow('Aken')
cv2.createTrackbar("Kerneli suurus", "Aken", kernel2, 50, kernel)

video_capture_device = cv2.VideoCapture(0)
while True:
    algus =  time.time()
    kulunud_aeg = algus - lopp
    fps  = 1 / kulunud_aeg
    lopp = algus
    Ret, frame = video_capture_device.read()
    frame = cv2.resize(frame, (0,0), fx=0.5, fy=0.5)
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    lowerLimits = np.array([lB, lG, lR])
    upperLimits = np.array([hB, hG, hR])
    thresholded = cv2.inRange(frame, lowerLimits, upperLimits)
    outimage = cv2.bitwise_and(frame, frame, mask = thresholded)
    
    algus2 =  time.time()
    kulunud_aeg2 = algus2 - lopp2
    fps2  = 1 / kulunud_aeg2
    median = cv2.medianBlur(frame,kernel2)
    lopp2 = algus2
    
    algus3 =  time.time()
    kulunud_aeg3 = algus3 - lopp3
    fps3  = 1 / kulunud_aeg3
    blur = cv2.bilateralFilter(frame,9,75,75)
    lopp3 = algus3
    
    
    
    
    cv2.putText(outimage, str(int(fps)), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(median, str(int(fps2)), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(blur, str(int(fps3)), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow('Tavaline', outimage)
    cv2.imshow('Mediaan', median)
    cv2.imshow('Blurred', blur)
    













    if cv2.waitKey(1) & 0xFF == ord('q'):
        break



f = open("trackbar_defaults.txt","w+")
values = [lB,lG,lR,hB,hG,hR]
for i in values:
    f.write(str(i))
    f.write("\n")
f.close()

    


# When everything done, release the capture
print('closing program')
cap.release()
cv2.destroyAllWindows()

