# -*- coding: utf-8 -*-
"""
Created on Wed Mar 16 09:37:21 2022

To create a simple dataset of QR code pictures

@author: Alec
"""

import cv2
import numpy as np
import os

def qrshow(img,win = 'qr',resize = (600,600), wait = 0):
    # Input string for win, ndarray for img
    cv2.namedWindow(win,cv2.WINDOW_NORMAL)
    cv2.imshow(win,img)
    cv2.resizeWindow(win,resize[0],resize[1])
    cv2.waitKey(wait)

def qrrot(img, theta, center = None, scale = 1.0, back = 255):
    # assumes image is BW
    if not center:
        center = (img.shape[0]/2,img.shape[1]/2)
    
    
    M = cv2.getRotationMatrix2D(center,theta,scale)
    
    # Rotate image and fill background space in as desired
    rimg = cv2.warpAffine(img,M,img.shape,borderValue=back)
    
    return rimg
    

## Source image:
# path = 'C:\Users\Alec\Documents\NSTAR\Software\Python\Depth and Pose Estimation - ODROID+D435\1_ PoseEst\example_scripts\4_QR_PoseEst_Library\QR-Pose-Position-master\easyImages_Engl'
path = '.\\easyImages_Engl\\rollOnly\\whiteBack'
sfname = 'qrcode_0.png'
back_color = 255 # Specify background color

qr = cv2.imread(os.path.join(path,sfname))
qr = cv2.cvtColor(qr, cv2.COLOR_BGR2GRAY)

# Assuming original image is square, we avoid cutoff due to rotation by adding a factor of (sqrt2)-1 of the image's original width 
slack = round((np.sqrt(2)-1)*qr.shape[0]/2) + 2

# Add this slack in the form of extra white pixels to top, bottom, left, right
qr = cv2.copyMakeBorder(qr,slack,slack,slack,slack, cv2.BORDER_CONSTANT, value = back_color)
qrshow(qr)

## Make some in plane rotations
thetas = np.linspace(0,360,num=120)
basename = 'qrcode_'
for theta in thetas:
    theta = round(theta)
    fname = basename+'%d.png'%theta
    
    # Specify background color
    rqr = qrrot(qr,theta,back=back_color)    
    
    # qrshow(rqr,wait = 1) # Want to transition quickly between them
    
    cv2.imwrite(os.path.join(path,fname),rqr)

cv2.destroyAllWindows()