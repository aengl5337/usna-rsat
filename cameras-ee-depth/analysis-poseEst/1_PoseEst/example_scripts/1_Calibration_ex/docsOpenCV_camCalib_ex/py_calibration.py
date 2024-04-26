# -*- coding: utf-8 -*-
"""
Created on Tue Apr  6 17:26:17 2021

Camera Calibration
Pulled from: https://github.com/abidrahmank/OpenCV2-Python-Tutorials/blob/master/source/py_tutorials/py_calib3d/py_calibration/py_calibration.rst

@author: m211740
"""

import numpy as np
import cv2
import glob

"""
So to find pattern in chess board, we use the function, cv2.findChessboardCorners(). 
We also need to pass what kind of pattern we are looking, like 8x8 grid, 5x5 grid etc. 
In this example, we use 7x6 grid. (Normally a chess board has 8x8 squares and 7x7 internal corners). 
It returns the corner points and retval which will be True if pattern is obtained. 
These corners will be placed in an order (from left-to-right, top-to-bottom)

Once we find the corners, we can increase their accuracy using cv2.cornerSubPix(). 
e can also draw the pattern using cv2.drawChessboardCorners(). 
All these steps are included in below code:
"""

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('*.jpg')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,6),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (7,6), corners2,ret)
        cv2.imshow('img',img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

"""
So now we have our object points and image points we are ready to go for calibration. 
For that we use the function, cv2.calibrateCamera(). 
It returns the camera matrix, distortion coefficients, rotation and translation vectors etc.
"""

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

"""
We have got what we were trying. Now we can take an image and undistort it. 
OpenCV comes with two methods, we will see both. 
But before that, we can refine the camera matrix based on a free scaling parameter using cv2.getOptimalNewCameraMatrix(). 
If the scaling parameter alpha=0, it returns undistorted image with minimum unwanted pixels. 
So it may even remove some pixels at image corners. If alpha=1, all pixels are retained with some extra black images. 
It also returns an image ROI which can be used to crop the result.

So we take a new image (left12.jpg in this case. That is the first image in this chapter)
"""

img = cv2.imread('left12.jpg')
h,  w = img.shape[:2]
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

"""
1. Using cv2.undistort()
This is the shortest path. Just call the function and use ROI obtained above to crop the result.
"""
# undistort
dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

# crop the image
x,y,w,h = roi
dst = dst[y:y+h, x:x+w]
cv2.imwrite('calibresult.png',dst)

"""
2. Using remapping
This is curved path. First find a mapping function from distorted image to undistorted image. 
Then use the remap function.
"""
# undistort
mapx,mapy = cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(w,h),5)
dst = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)

# crop the image
x,y,w,h = roi
dst = dst[y:y+h, x:x+w]
cv2.imwrite('calibresult.png',dst)

"""
Both the methods give the same result!
"""

"""
Re-projection error gives a good estimation of just how exact is the found parameters. 
This should be as close to zero as possible. 
Given the intrinsic, distortion, rotation and translation matrices, 
we first transform the object point to image point using cv2.projectPoints(). 
Then we calculate the absolute norm between what we got with our transformation 
and the corner finding algorithm. To find the average error we calculate the 
arithmetical mean of the errors calculate for all the calibration images.
"""
tot_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    tot_error += error

print("Avg error: ", tot_error/len(objpoints))