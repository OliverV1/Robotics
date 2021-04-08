#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
detector = cv2.SimpleBlobDetector_create()






#Working with image files stored in the same folder as .py file
#Load the image from the given location
nimi = "sample01.tiff"
img = cv2.imread(nimi)



#Load the image from the given location in greyscale
img_greyscale = cv2.imread(nimi, 0)

trackbar_value = 1


#Thresholding the image (Refer to opencv.org for more details)
def updateValue(new_value):
    global trackbar_value
    trackbar_value = new_value
    return
blobparams = cv2.SimpleBlobDetector_Params()
blobparams.filterByArea = False
blobparams.filterByCircularity = False
blobparams.minDistBetweenBlobs = 200
blobparams.filterByInertia = False
blobparams.filterByConvexity = False





# määrab ära pallida thresholdi
#kuvab pilti
cv2.namedWindow('Threshold')
cv2.namedWindow('Originaal')







cv2.createTrackbar("Example trackbar", "Threshold", trackbar_value, 100, updateValue)
while True:
    ret, thresh = cv2.threshold(img_greyscale, trackbar_value, 255, cv2.THRESH_BINARY)
    keypoints = detector.detect(thresh)
    obama = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    for i in range(len(keypoints)):
        nimi1 = "X = " + str(keypoints[i].pt[0]) + " Y= " + str(keypoints[i].pt[1]) 
        cv2.putText(obama, nimi1, (int(keypoints[i].pt[0]),int(keypoints[i].pt[1])), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
    cv2.imshow("Originaal", obama)
    cv2.imshow('Threshold', thresh)

    if cv2.waitKey(1) & 0xFF == ord('q'):

        break
    
#Display the images




cv2.waitKey(0)
cv2.destroyAllWindows()

