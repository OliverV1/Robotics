#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import time
lopp = 0
lB = 0
lG = 0
lR = 0
hB = 255
hG = 255
hR = 255
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
cv2.namedWindow('Aken')
blobparams = cv2.SimpleBlobDetector_Params()
blobparams.filterByArea = True
blobparams.filterByCircularity = False
blobparams.minDistBetweenBlobs = 800
blobparams.filterByInertia = False
blobparams.filterByConvexity = False
blobparams.minArea = 300
blobparams.maxArea = 10000
detector = cv2.SimpleBlobDetector_create(blobparams)

cv2.createTrackbar("Lower B", "Aken", lB, 255, updateValue)
cv2.createTrackbar("Lower G", "Aken", lG, 255, updateValue1)
cv2.createTrackbar("Lower R", "Aken", lR, 255, updateValue2)
cv2.createTrackbar("Higher B", "Aken", hB, 255, updateValue3)
cv2.createTrackbar("Higher G", "Aken", hG, 255, updateValue4)
cv2.createTrackbar("Higher R", "Aken", hR, 255, updateValue5)


video_capture_device = cv2.VideoCapture(0)

while True:
  algus =  time.time()
  kulunud_aeg = algus - lopp
  fps  = 1 / kulunud_aeg
  lopp = algus
  Ret, frame = video_capture_device.read()
  lowerLimits = np.array([lB, lG, lR])
  upperLimits = np.array([hB, hG, hR])
  thresholded = cv2.inRange(frame, lowerLimits, upperLimits)
  
  outimage = cv2.bitwise_and(frame, frame, mask = thresholded)
  thresholded = cv2.bitwise_not(thresholded)
  cv2.putText(outimage, str(int(fps)), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
  keypoints = detector.detect(thresholded)
  obama = cv2.drawKeypoints(outimage, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


  cv2.imshow('Processe2d', obama)
  

  if cv2.waitKey(1) & 0xFF == ord('q'):
    break
print('closing program')
video_capture_device.release()
cv2.destroyAllWindows()

