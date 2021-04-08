#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import time
lopp = 0
# Open the camera
cap = cv2.VideoCapture(0)
while True:
  # Read the image from the camera
  algus =  time.time()
  kulunud_aeg = algus - lopp
  fps  = 1 / kulunud_aeg
  lopp = algus
  
  ret, frame = cap.read()
  
  # Write some text onto the frame
 

  # Show this image on a window named "Original"

  

 
  cv2.putText(frame, str(int(fps)), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
  cv2.imshow('Original', frame)# Quit the program when 'q' is pressed

  if cv2.waitKey(1) & 0xFF == ord('q'):
    break

# When everything done, release the capture
print('closing program')
cap.release()
cv2.destroyAllWindows()

