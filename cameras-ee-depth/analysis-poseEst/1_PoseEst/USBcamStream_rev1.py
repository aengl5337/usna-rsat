# -*- coding: utf-8 -*-
"""
Created on Mon Feb 22 14:58:08 2021

@author: m211740
"""

#! /usr/bin/python3
import numpy as np
import cv2
from imutils.video import WebcamVideoStream
import sys

cam_index=0

# open the video stream 
camera = WebcamVideoStream(cam_index).start()

# index list (with external webcam plugged in)
cam_names=['D4**','Webcam','N/A'] # Note that generic usb webcams become index=1

while(True):
    cam_name=cam_names[cam_index]
    # Capture frame-by-frame
    img = camera.read()
	# Display the resulting frame
    cv2.imshow(cam_name,img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
camera.stop()
cv2.destroyAllWindows()
