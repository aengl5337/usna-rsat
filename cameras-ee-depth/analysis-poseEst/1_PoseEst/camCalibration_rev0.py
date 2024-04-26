# -*- coding: utf-8 -*-
"""
Created on Thu Sep  2 19:34:13 2021
REV0: 06APR21 by Alec Engl copied+adapted from niconielsen32 (https://github.com/niconielsen32/ComputerVision/blob/master/cameraCalibration.py)
Dr. Kutzer's calibration chessboard used


@author: engl
"""

import numpy as np
import cv2
import glob



################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################

chessboardSize = (6,7) # Remember, opencv uses (columns, rows) coordinates (aka (x,y))

#frameSize = (1440,1080)
frameSize = (480,640) # for hp webcam



# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2) #generates locations for each internal point in the chessboard


# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.


#images = glob.glob('*.png')
#images = glob.glob('*.png')

cam=cv2.VideoCapture(0)
#ret, img=cam.read() #pull frame from webcam

#for image in images:
currFrame=0
currGoodFrame=0
while currGoodFrame<21:
    
    retw, img=cam.read() #pull frame from webcam
    #img = cv2.imread(image)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, chessboardSize, None)

    # If found, add object points, image points (after refining them)
    if ret == True:

        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, chessboardSize, corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(1000)
        
        currGoodFrame+=1
        
    print(currFrame)
    currFrame+=1
#
##img = cv2.imread(image)
#gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#
## Find the chess board corners
#ret, corners = cv2.findChessboardCorners(gray, chessboardSize, None)
#
## If found, add object points, image points (after refining them)
#if ret == True:
#
#    objpoints.append(objp)
#    corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
#    imgpoints.append(corners)
#
#    # Draw and display the corners
#    cv2.drawChessboardCorners(img, chessboardSize, corners2, ret)
#    cv2.imshow('img', img)
#    cv2.waitKey(1000)


cv2.destroyAllWindows()




############## CALIBRATION #######################################################

ret, cameraMatrix, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, frameSize, None, None)


############## UNDISTORTION #####################################################

#img = cv2.imread('cali5.png')
h,  w = img.shape[:2]
newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))



# Undistort
dst = cv2.undistort(img, cameraMatrix, dist, None, newCameraMatrix)

# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv2.imwrite('caliResult1.png', dst)



# Undistort with Remapping
mapx, mapy = cv2.initUndistortRectifyMap(cameraMatrix, dist, None, newCameraMatrix, (w,h), 5)
dst = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv2.imwrite('caliResult2.png', dst)




# Reprojection Error
mean_error = 0

for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    mean_error += error

print( "total error: {}".format(mean_error/len(objpoints)) )