# -*- coding: utf-8 -*-
"""
Created on 09SEP2021
REV0: 09SEP21 by Alec Engl, modified from original example 
>'line()' requires integer point tuple inputs, as pixels aren't continuous

@author: m211740
"""

import cv2
import numpy as np
import os
import glob
import math
import QRPoints_EnglMod as QRPoints

################ FUNCTION DEFINITIONS #############################

#This function draws the axis on the QR Code
def draw(img,corners,imgpts):
        """
        cv.line(	img, pt1, pt2, color[, thickness[, lineType[, shift]]]	) ->	img
            Parameters
            img	Image.
            pt1	First point of the line segment.
            pt2	Second point of the line segment.
            color	Line color.
            thickness	Line thickness.
            lineType	Type of the line. See LineTypes.
            shift	Number of fractional bits in the point coordinates.
        
        """
        corner=(int(corners[0].ravel()[0]),int(corners[0].ravel()[1]))
        img = cv2.line(img, corner, (int(imgpts[0].ravel()[0]),int(imgpts[0].ravel()[1])), (255,0,0), 5)
        """
        09SEP bug:
          error: OpenCV(4.5.3) :-1: error: (-5:Bad argument) in function 'line'
            > Overload resolution failed:
            >  - Can't parse 'pt1'. Sequence item with index 0 has a wrong type
            >  - Can't parse 'pt1'. Sequence item with index 0 has a wrong type
            
            type(corner) = tuple
            
            ANSWER: make sure all coords are 'int'
            
        """
        img = cv2.line(img, corner, (int(imgpts[1].ravel()[0]),int(imgpts[1].ravel()[1])), (0,255,0), 5)
        img = cv2.line(img, corner, (int(imgpts[2].ravel()[0]),int(imgpts[2].ravel()[1])), (0,0,255), 5)
        return img

def calib_cam(cams_idx = 1,recalibrate = False): ## * Make existence of calib file a prereq
    # # Camera matrix
    # fx,fy
    # cx,cy
    # mtx=np.array([[fx, 0, cx],
    #            [0, fy, cy],
    #            [0, 0, 1]])
    # # Distortion coefficients
    # k1,k2,k3
    # p1,p2
    # dist=np.array([k1, k2, p1, p2, k3])
    #Load npz file with camera calibration parameters
    
    # Provide some proprietary camera examples 
    cams = ['iPhoneCam','surface_back_webcam','D435_RGB']
    B_fname = cams[cams_idx] + '.npz'
    
    if cams_idx == 0:
        with np.load(B_fname) as X:
            mtx,dist,_,_=[X[i] for i in('mtx','dist','rvecs','tvecs')]
            newmtx = mtx #** Temporary fix for the iPhone pics
    else:
        if os.path.exists(B_fname) or not(recalibrate):
          with np.load(B_fname) as X:
              mtx, newmtx, dist, _, _ = [X[i] for i in ('mtx','newmtx','dist','rvecs','tvecs')]#* do I need these? ,'rvecs','tvecs'
        else:
            chessboardSize = (8-1,8-1)
            
            # termination criteria
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) #**this is the original one        
            # criteria=(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER)
            
            # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
            objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
            objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)
            
            
            # Arrays to store object points and image points from all the images.
            objpoints = [] # 3d point in real world space
            imgpoints = [] # 2d points in image plane.
            
            
            images = glob.glob(r'.\..\..\..\example_img\surface_back_webcam_img\*.jpg')
            
            for image in images:
            
                img = cv2.imread(image)
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
                # Find the chess board corners
                ret, corners = cv2.findChessboardCorners(gray, chessboardSize, None)
            
                # If found, add object points, image points (after refining them)
                if ret == True:
            
                    objpoints.append(objp)
                    corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
                    imgpoints.append(corners)
            
                    # Draw and display the corners
                    imgDetect=img
                    cv2.drawChessboardCorners(imgDetect, chessboardSize, corners2, ret)
                    
                    if cams_idx == 1:
                        imgDetectS = cv2.resize(imgDetect, (960, 540)) # **Image too large to display
                    else:
                        imgDetectS = imgDetect
                        
                    cv2.imshow('ChessDetected', imgDetectS)
                    cv2.waitKey(0)
            
            ################ USE DETECTION DATA TO CALIBRATE CAMERA, COMPUTE INTRINSICS #############################
            
            # Use selected image to get/format frame size
            frameSize = np.shape(img)
            frameSize = (frameSize[1],frameSize[0])
            cv2.destroyAllWindows()
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, frameSize, None, None)
            
            ################ REFINE CAMERA MATRIX to better UNDISTORT IMAGE #############################
            
            # Refining the camera matrix using parameters obtained by calibration
            newmtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, frameSize, 1, frameSize)
            
            ################ SAVE INTRINSIC PARAMS FOR NEXT TIME #############################
            
            vals_to_save = {'mtx':mtx,'newmtx':newmtx,'dist':dist,'rvecs':rvecs,'tvecs':tvecs}
            np.savez(B_fname, **vals_to_save)
        
    return mtx,newmtx,dist

################ CONSTANTS/FLAGS/GEOMETRY #############################

# criteria **
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


# QR code detection - openCV ** remove this after debugging
detector = cv2.QRCodeDetector()

if useCamStream:
    if cams_idx == 1: # Using back Surface webcam
        cam=cv2.VideoCapture(1) #Seems to alternately choose front or back webcam on surface
        while(True):
            ret, img=cam.read()
            print("Image taken")
            # cv2.imshow("Image taken",img)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
            # dst2 = undistort(img,mtx,newmtx,dist)
            # print("Image undistorted")
            
            # img_variants = [img,dst2]
            # img_str = ['Distorted', 'Undistorted']
            
            img_variants = [img]
            img_str = ['Distorted?'] #*?? does this code handle this?
            
            i = 0
            for each_img in img_variants:
                
                # Evan's detector:
                # # Detects cube side 'data' of QR code, and establishes the coordinates of the bounding box 
                # data, bbox, _ = detector.detectAndDecode(each_img)
                
                # # draws that blue box around the QR code and overlays the image with the data. Box coordinates in tuple(bbox)
                # if(bbox is not None):
                #     for i in range(len(bbox)):
                #         cv2.line(each_img, (int(bbox[i][0][0]),int(bbox[i][0][1])), (int(bbox[(i+1) % len(bbox)][0][0]),int(bbox[(i+1) % len(bbox)][0][1])), color=(255,
                #                  0, 0), thickness=2)
                #     cv2.putText(each_img, data, (int(bbox[0][0][0]), int(bbox[0][0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX,
                #                 1, (255, 250, 120), 2)
                
                # Alec's detector:
                # Find barcodes and QR codes (accepts img or path)
                imagePoints,success=QRPoints.getPoints(each_img)
        
                if not success:
                        imagePoints,success=QRPoints.getPoints(each_img,7,5)
                        ## * PUT condition if no success
                
                if success:
                    #Now use solvePnP to get the rotation and translation vectors
                    ret,rvecs,tvecs=cv2.solvePnP(objectPoints,imagePoints,mtx,dist)
            
                    #get projected points for drawing axes in image
                    # I.e., based on the pose determined, figure out where the basis unit vectors ended up
                    projectedPoints,_=cv2.projectPoints(axis,rvecs,tvecs,mtx,dist)
            
                    # Draw our axes on the image for testing
                    each_img=draw(each_img,imagePoints,projectedPoints)
            
                    # Get rotation matrix from object coordinates to camera coordinates
                    # Converts eigenaxis to matrix
                    rotationMat=cv2.Rodrigues(rvecs)[0]
            
                    #Get camera position
                    camPos=-np.matrix(rotationMat).T * np.matrix(tvecs) ## Use * and not @ since matrix not ndarray
            
                    #create the projection matrix
                    projMat=np.hstack((rotationMat,tvecs))
            
                    #get euler angles from projection matrix
                    eul=-cv2.decomposeProjectionMatrix(projMat)[6]
            
                    yaw=eul[1,0]
                    pitch=(eul[0,0]+(90))*math.cos(eul[1,0])
                    roll=(-(90)-eul[0,0])*math.sin(eul[1,0]) +eul[2,0]
            
                    # #Add text
                    # r0=200
                    # dr=50
                    # c0=10
                    # font=cv2.FONT_HERSHEY_DUPLEX
                    # cv2.putText(each_img,("X: " + str(camPos[0][0][0])),(c0,r0+dr),font,1,(255,255,255),2,cv2.LINE_AA)
                    # cv2.putText(each_img,("Y: " + str(camPos[1][0])),(c0,r0+2*dr),font,1,(255,255,255),2,cv2.LINE_AA)
                    # cv2.putText(each_img,("Z: " + str(camPos[2][0])),(c0,r0+3*dr),font,1,(255,255,255),2,cv2.LINE_AA)
                    # cv2.putText(each_img,("Roll: " + str(round(roll,2))),(c0,r0+5*dr),font,1,(255,255,255),2,cv2.LINE_AA)
                    # cv2.putText(each_img,("Pitch: " + str(round(pitch,2))),(c0,r0+6*dr),font,1,(255,255,255),2,cv2.LINE_AA)
                    # cv2.putText(each_img,("Yaw: " + str(round(yaw,2))),(c0,r0+4*dr),font,1,(255,255,255),2,cv2.LINE_AA)
                
                # Either way, whether detected or not, display image
                
                #Display image, resize
                cv2.namedWindow(img_str[i],cv2.WINDOW_NORMAL)
                each_imgS = cv2.resize(each_img, (960, 540)) # **Image too large to display
                cv2.imshow(img_str[i],each_imgS)
                cv2.resizeWindow(img_str[i],600,600)
                # cv2.waitKey(0)
                # cv2.destroyAllWindows()
                                
                i+=1
                
                # This doesn't stop anything when (0)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                """
                ord('q') returns the Unicode code point of q
                cv2.waitkey(1) returns a 32-bit integer corresponding to the pressed key
                & 0xFF is a bit mask which sets the left 24 bits to zero, because ord() returns a value betwen 0 and 255, since your keyboard only has a limited character set
                Therefore, once the mask is applied, it is then possible to check if it is the corresponding key.
                """
        cam.release()
    # if cams_idx == 2: # Using D435**
    
# If not streaming AND using cam=0 (the provided iPhone pics)
elif cams_idx == 0:
    #Load the images
    # images=glob.glob('markerImages/*.JPG')
    # images=glob.glob('markerImages_Engl/*.JPG')
    images=glob.glob('easyImages_Engl\\rollOnly\\*.png')
    
    for fname in images:
            print("File name: " + str(fname))
    
            image=cv2.imread(fname)
            
            
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
    
            imagePoints,success=QRPoints.getPoints(fname)
    
            if not success:
                    imagePoints,success=QRPoints.getPoints(fname,7,5)
    
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