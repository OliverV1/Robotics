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
    Ret, frame = video_capture_device.read()
    frame = cv2.resize(frame, (0,0), fx=0.5, fy=0.5)
    lowerLimits = np.array([lB, lG, lR])
    upperLimits = np.array([hB, hG, hR])
    thresholded = cv2.inRange(frame, lowerLimits, upperLimits)
    outimage = cv2.bitwise_and(frame, frame, mask = thresholded)
    kernel = np.ones((kernel2,kernel2),np.uint8)
    erosion = cv2.erode(thresholded,kernel,iterations = 1)
    dilation = cv2.dilate(thresholded,kernel,iterations = 1)
    erosion_dilation = cv2.dilate(erosion,kernel,iterations = 1)
    dilation_erosion = cv2.dilate(dilation,kernel,iterations = 1)
    
    cv2.imshow('Erosion', erosion)
    cv2.imshow('dilation', dilation)
    cv2.imshow('erosion_dilation', erosion_dilation)
    cv2.imshow('dilation_erosion', dilation_erosion)

    













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
cv2.destroyAllWindows()

