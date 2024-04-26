# -*- coding: utf-8 -*-
"""
# NOTE: commented asterisks indicate fix points
REV0: 16MAR22 by Alec Engl
# To test QR detectors/pose estimators on QR data

@author: m211740
"""

import cv2
import numpy as np
import os
import glob
import math
from qrfun import * # Imports all support functions

################ CONSTANTS/FLAGS/GEOMETRY #############################

# criteria for detection **
#3D point array
objectPoints=np.zeros((6,3),dtype=np.float32)
#List points in the pattern.png image. The coordinates correspond to cm**
pointList=[[0,0,0],[2.464,2.464,0],[0,8.8,0],[2.464,6.336,0],[8.8,0,0],[6.336,2.464,0]]
#Axis to draw on the image later. Axis will be as long as the QRCode
axis = np.float32([[8.8,0,0], [0,8.8,0], [0,0,8.8]]).reshape(-1,3)

#Add the points into the 3D point array##
for i,x in zip(pointList,range(0,len(pointList))):
        objectPoints[x]=i

# Determine intrinsic camera parameters
recalibrate = False
cams_idx = 0# **MODIFY BASED ON PREFERENCE**... delete this
mtx, newmtx, dist = calib_cam(cams_idx, recalibrate)

# Declare method of analysis
useCamStream = False

# assert (cams_idx!=0)&(useCamStream) # iPhone camera was used for stillframe examples ONLY

cust_classical = Detector()
OCV_classical = Detector()
OCV_CNN = Detector()

## Assign specific methods to each detector instance
# Each must take input images and output 
cust_classical.method = classQRdetect
OCV_classical.method = cv2.QRCodeDetector.detectAndDecode
OCV_CNN.method = cv2.QRCodeDetector
            
            
            # Evan's detector:
            # # Detects cube side 'data' of QR code, and establishes the coordinates of the bounding box 
            # data, bbox, _ = detector.detectAndDecode(image)
            
            # # draws that blue box around the QR code and overlays the image with the data. Box coordinates in tuple(bbox)
            # if(bbox is not None):
            #     for i in range(len(bbox)):
            #         cv2.line(image, (int(bbox[i][0][0]),int(bbox[i][0][1])), (int(bbox[(i+1) % len(bbox)][0][0]),int(bbox[(i+1) % len(bbox)][0][1])), color=(255,
            #                   0, 0), thickness=2)
            #     cv2.putText(image, data, (int(bbox[0][0][0]), int(bbox[0][0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX,
            #                 1, (255, 250, 120), 2)
    
            
    
            # #Now use solvePnP to get the rotation and translation vectors
            # ret,rvecs,tvecs=cv2.solvePnP(objectPoints,imagePoints,mtx,dist)
    
            # #get projected points for drawing axis in image
            # projectedPoints,_=cv2.projectPoints(axis,rvecs,tvecs,mtx,dist)
    
            # #Draw our axis on the image for testing
            # image=draw(image,imagePoints,projectedPoints)
    
            # # Get rotation matrix from object coordinates to camera coordinates
            # rotationMat=cv2.Rodrigues(rvecs)[0]
    
            # #Get camera position
            # camPos=-np.matrix(rotationMat).T * np.matrix(tvecs)
    
            # #create the projection matrix
            # projMat=np.hstack((rotationMat,tvecs))
    
            # #get euler angles from projection matrix
            # eul=-cv2.decomposeProjectionMatrix(projMat)[6]
    
            # yaw=eul[1,0]
            # pitch=(eul[0,0]+(90))*math.cos(eul[1,0])
            # roll=(-(90)-eul[0,0])*math.sin(eul[1,0]) +eul[2,0]
    
            # #Add text
            # font=cv2.FONT_HERSHEY_DUPLEX
            # cv2.putText(image,("X: " + str(camPos[0][0][0])),(10,200),font,4,(255,255,255),2,cv2.LINE_AA)
            # cv2.putText(image,("Y: " + str(camPos[1][0])),(10,350),font,4,(255,255,255),2,cv2.LINE_AA)
            # cv2.putText(image,("Z: " + str(camPos[2][0])),(10,500),font,4,(255,255,255),2,cv2.LINE_AA)
            # cv2.putText(image,("Roll: " + str(round(roll,2))),(10,950),font,4,(255,255,255),2,cv2.LINE_AA)
            # cv2.putText(image,("Pitch: " + str(round(pitch,2))),(10,800),font,4,(255,255,255),2,cv2.LINE_AA)
            # cv2.putText(image,("Yaw: " + str(round(yaw,2))),(10,650),font,4,(255,255,255),2,cv2.LINE_AA)
            
            #Display image, resize
            cv2.namedWindow('img',cv2.WINDOW_NORMAL)
            cv2.imshow('img',image)
            cv2.resizeWindow('img',600,600)
            cv2.waitKey(1)
            # cv2.destroyAllWindows()
else:
    print("No example images available for selected camera")

cv2.destroyAllWindows()