# -*- coding: utf-8 -*-
"""
Created on Mon Sep  6 10:33:19 2021

Mixed from OpenCV and Nielsen tutorials (basically the same)

Calibration sequence over several example images provides the tools to undistort target images and estimate pose

Credit: https://github.com/niconielsen32/ComputerVision/blob/master
        https://github.com/abidrahmank/OpenCV2-Python-Tutorials/blob/master/source/py_tutorials/py_calib3d/py_pose/py_pose.rst

@author: Alec
"""

import numpy as np
import cv2
import os
import glob

################ FUNCTION DEFINITIONS #############################


def draw(img, corners, imgpts):

    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 10)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 10)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 10)

    return img


def drawBoxes(img, corners, imgpts):

    imgpts = np.int32(imgpts).reshape(-1,2)

    # draw ground floor in green
    img = cv2.drawContours(img, [imgpts[:4]],-1,(0,255,0),-3)

    # draw pillars in blue color
    for i,j in zip(range(4),range(4,8)):
        img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)

    # draw top layer in red color
    img = cv2.drawContours(img, [imgpts[4:]],-1,(0,0,255),3)

    return img

def calib_cam():
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
        
    if os.path.exists('B.npz'):
      with np.load('B.npz') as X:
          mtx, newmtx, dist = [X[i] for i in ('mtx','newmtx','dist')]#* do I need these? ,'rvecs','tvecs'
    else:
        chessboardSize = (8-1,8-1)
        
        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
        
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
        objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)
        
        
        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.
        
        
        #images = glob.glob(r'.\..\..\camCalibration\nielsen_calib_ex\cameraCalibration\calibration\*.png')
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
                imgDetectS = cv2.resize(imgDetect, (960, 540)) # Image too large to display
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
        
        vals_to_save = {'mtx':mtx,'newmtx':newmtx,'dist':dist}
        np.savez('B.npz', **vals_to_save)
        
    return mtx,newmtx,dist

################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################

mtx,newcameramtx,dist = calib_cam()

################ REFINE CAMERA MATRIX AND UNDISTORT IMAGE #############################

# # criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# objp = np.zeros((24*17,3), np.float32)
# objp[:,:2] = np.mgrid[0:24,0:17].T.reshape(-1,2)
# axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
# axisBoxes = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
#                    [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])

# # Method 1 to undistort the image
# dst1 = cv2.undistort(img, mtx, dist, None, newcameramtx)

# # Method 2 to undistort the image
# mapx,mapy=cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,frameSize,5)
# dst2 = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)
# # Displaying the undistorted image
# dst1S = cv2.resize(dst1, (960, 540)) # Image too large to display at its normal scale, resize for display (not calculations)
# dst2S = cv2.resize(dst2, (960, 540)) # Image too large to display at its normal scale, resize for display (not calculations)
# cv2.imshow("undistorted image, M1",dst1S)
# cv2.waitKey(0)
# cv2.imshow("undistorted image, M2",dst2S)
# cv2.waitKey(0)
# cv2.destroyAllWindows()


# # for image in glob.glob('undistorted*.png'):

#     # img = cv2.imread(image)
#     # gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
#     # ret, corners = cv2.findChessboardCorners(gray, chessboardSize,None)

#     # if ret == True:

#     #     corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1), criteria)

#     #     # Find the rotation and translation vectors.
#     #     ret, rvecs, tvecs = cv2.solvePnP(objp, corners2, mtx, dist)

#     #     # Project 3D points to image plane
#     #     imgpts, jac = cv2.projectPoints(axisBoxes, rvecs, tvecs, mtx, dist)

#     #     img = drawBoxes(img,corners2,imgpts)
#     #     cv2.imshow('img',img)

#     #     k = cv2.waitKey(0) & 0xFF
#     #     if k == ord('s'):
#     #         cv2.imwrite('pose'+image, img)

################ FIND POSE OF CHESSBOARD IN UNDISTORTED IMAGE #############################

# Redetect the chessboard in an image for simplicity
# Method 2 to undistort the image, THIS TIME WITHOUT THE CHESSBOARD DETECTION GRAPHIC

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
useWebcamStream = True
axisBoxes = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
                        [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])

if useWebcamStream:
    cam = cv2.VideoCapture(1)
    chessboardSize = (5,5)    
    _, img = cam.read() #Take a sample image to read frame size
    frameSize = np.shape(img)
    frameSize = (frameSize[1],frameSize[0])
    
    objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
    objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)
    
    mapx,mapy=cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,frameSize,5)
    
    while True:
        _, img = cam.read()
        print("Image taken")
        dst2 = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)
        print("Image undistorted")
        
        img_variants = [img,dst2]
        img_str = ['Distorted', 'Undistorted']
        
        i = 0
        for each_img in img_variants:
            gray = cv2.cvtColor(each_img,cv2.COLOR_BGR2GRAY)
            
            print("Searching for chessboard")
            ret, corners = cv2.findChessboardCorners(gray, chessboardSize,None)
            
            if ret == True:
                print("Chessboard detected")
                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1), criteria)
            
                # Find the rotation and translation vectors.
                ret, rvecs, tvecs = cv2.solvePnP(objp, corners2, mtx, dist)
            
                # Project 3D points to image plane
                imgpts, jac = cv2.projectPoints(axisBoxes, rvecs, tvecs, mtx, dist)
            
                each_img = drawBoxes(each_img,corners2,imgpts)
                #imgS = cv2.resize(img, (960, 540)) # Image too large to display at its normal scale, resize for display (not calculations)
                
            cv2.imshow(img_str[i],each_img)
            i+=1
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break

else:
    img = cv2.imread(image)
    mapx,mapy=cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,frameSize,5)
    dst2 = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)
    gray = cv2.cvtColor(dst2,cv2.COLOR_BGR2GRAY)
    objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
    objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)
    
    ret, corners = cv2.findChessboardCorners(gray, chessboardSize,None)
    
    if ret == True:
    
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1), criteria)
    
        # Find the rotation and translation vectors.
        ret, rvecs, tvecs = cv2.solvePnP(objp, corners2, mtx, dist)
    
        # Project 3D points to image plane
        imgpts, jac = cv2.projectPoints(axisBoxes, rvecs, tvecs, mtx, dist)
    
        img = drawBoxes(dst2,corners2,imgpts)
        imgS = cv2.resize(img, (960, 540)) # Image too large to display at its normal scale, resize for display (not calculations)
        cv2.imshow('Pose Est',imgS)
    
        k = cv2.waitKey(0) & 0xFF
        if k == ord('s'):
            cv2.imwrite('pose'+image, img)


cv2.destroyAllWindows()