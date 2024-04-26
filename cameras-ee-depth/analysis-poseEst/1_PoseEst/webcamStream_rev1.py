# -*- coding: utf-8 -*-
"""
Created on Sat Feb 20 20:48:06 2021

@author: m211740
"""

import cv2
import numpy as np

cam=cv2.VideoCapture(0)

currFrame=0
while(True):
    ret, frame=cam.read()
    cv2.imshow('hey',frame)
    """
    ord('q') returns the Unicode code point of q
cv2.waitkey(1) returns a 32-bit integer corresponding to the pressed key
& 0xFF is a bit mask which sets the left 24 bits to zero, because ord() returns a value betwen 0 and 255, since your keyboard only has a limited character set
Therefore, once the mask is applied, it is then possible to check if it is the corresponding key.
"""
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    currFrame+=1
    
cam.release()
cv2.destroyAllWindows()