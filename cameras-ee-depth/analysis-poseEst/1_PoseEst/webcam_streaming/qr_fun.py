# -*- coding: utf-8 -*-
"""
Created on Tue Apr  6 16:53:47 2021

Scanner and poseEst functions derived from adjacent directories

Includes static image testing if launched as main

@author: m211740
"""

from __future__ import print_function
import pyzbar.pyzbar as pyzbar
import numpy as np
import cv2
import glob

# Load previously saved data, camera matrix and distortion coefficients from the previous calibration result.
# See py_calibration.py in adjacent directory

"""
Load stored camera calibration info
with np.load('B.npz') as X:
    mtx, dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]
"""

## CALIBRATION FUNCTIONS ##

def calib_cam():
    # Camera matrix
    fx=fy=1
    cx=cy=1
    A=np.array([[fx, 0, cx],
               [0, fy, cy],
               [0, 0, 1]])
    # Distortion coefficients
    k1=k2=k3=1
    p1=p2=1
    d=np.array([k1, k2, p1, p2, k3])
    
    return A,d

## SCANNING FUNCTIONS ##

def decode(im) :
    # Find barcodes and QR codes
    decodedObjects = pyzbar.decode(im)
    
    # Print results
    for obj in decodedObjects:
        print('Type : ', obj.type)
        print('Data : ', str(obj.data),'\n')
    
    # Return array of corner points, indexed by (points (tuples), corner number)
#    corners=
    return decodedObjects


# Use decode function's output points establish convex shape of detected object
def pull_convex_subpix(im,decodedObject):
    points = decodedObject.polygon

    # If the points do not form a quad, find convex hull
    # Output is 'pyzbar.locations.Point' type.  Has attributes Point.x and Point.y for 2D.  INTEGER, so should use some sub-pixel enhancement
    if len(points) > 4 :
      hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
      hull = list(map(tuple, np.squeeze(hull)))
    else :
      hull = points;

    # Draw the convex hull
    n = len(hull)
    for j in range(0,n):
        cv2.line(im, hull[j], hull[ (j+1) % n], (255,0,0), 3)

    # Display results
    cv2.imshow("Results", im);
    cv2.waitKey(0);

    """
    # Setup a very proprietarily-dimensioned ndarray for input into the later PoseEst fcn (see barcode-QRcodeScanner.py)
    # cornerSubPix and solvePnPransac expect corners left to right, top to bottom (starting in the northwest-most corner)
    # i.e., iterating over x then y (since (x,y) is analagous to (cols, rows), as used by cv2)
    Example output:
    array([[[130.3668 , 112.17349]],

       [[202.19786, 112.34112]],

       [[274.26697, 112.42536]],

       [[346.31247, 112.53036]],

       [[418.4071 , 112.52721]],

       [[490.3152 , 113.1975 ]],

       [[562.45844, 113.54444]],

       [[129.77803, 184.0072 ]],

       [[201.63144, 184.30667]],

       [[273.71198, 184.39333]],

       [[345.79443, 184.52519]],

       [[418.2046 , 184.62405]],

       [[490.28674, 185.12202]],

       [[562.25946, 185.68904]],

       [[129.60425, 255.7114 ]],

       [[201.57442, 256.17862]],

       [[273.44324, 256.35278]],

       [[345.42004, 256.50836]],

       [[417.70776, 256.66302]],

       [[489.6991 , 257.08755]],

       [[561.9869 , 257.56735]],

       [[129.3808 , 328.02972]],

       [[201.0869 , 327.96393]],

       [[273.2361 , 328.2866 ]],

       [[345.3298 , 328.45053]],

       [[417.3554 , 328.61798]],

       [[489.64062, 329.39944]],

       [[561.6056 , 329.608  ]],

       [[128.78883, 399.7566 ]],

       [[200.78296, 399.87564]],

       [[272.75262, 400.41013]],

       [[345.0424 , 400.58875]],

       [[417.31168, 400.6867 ]],

       [[489.47836, 401.43503]],

       [[561.53955, 401.66006]],

       [[128.59657, 471.5118 ]],

       [[200.84766, 472.14484]],

       [[272.7032 , 472.37375]],

       [[344.72372, 472.57486]],

       [[417.09363, 473.02698]],

       [[489.24945, 473.46442]],

       [[561.3684 , 473.79584]],

       [[128.52026, 543.2223 ]],

       [[200.36835, 543.6024 ]],

       [[272.51715, 544.27924]],

       [[344.55762, 544.4696 ]],

       [[416.6669 , 544.72406]],

       [[488.93546, 545.43195]],

       [[561.34247, 545.62317]]], dtype=float32)
    # pyzbar decode, polygon outputs points counterclockwise (starting in the northwest-most corner)
    # i.e., example below: 
        113 ,  287
        113 ,  351
        503 ,  350
        503 ,  286
    """
    corners= np.array([[[hull[0].x,hull[0].y],
                       [hull[3].x,hull[3].y],
                       [hull[1].x,hull[1].y],
                       [hull[2].x,hull[2].y]]], dtype=np.float32)

    
    # Not sure if grayscale is necessary, but was used in original
    gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    # Not sure if this needs to be edited
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    corners_subpix=cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
    
    return corners_subpix

## POSE ESTIMATION FUNCTIONS ##

def est_pose(im,decodedObjects):
    mtx,dist=calib_cam()
    for eachObj in decodedObjects:
        corners_subpix = pull_convex_subpix(im,eachObj)
        objp = np.array([[0.,0.,0.],[1.,0.,0.],
                         [1.,1.,0.],[0.,1.,0.]], dtype='float32')
        # Find the rotation and translation vectors.
##        rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners_subpix, mtx, dist) # ***10APR21: STOPPED AT ERROR ValueError: too many values to unpack (expected 3)***
        cv2.solvePnPRansac(objp, corners_subpix, mtx, dist)
##    return rvecs,tvecs # How to store rvecs/tvecs for multiple objects and return?
        print(corners_subpix)
        
    """
    # Specific to particular chessboard used in example
    objp = np.zeros((6*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
    
    # axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
    
    for fname in glob.glob('left*.jpg'):
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (7,6),None)
    
        if ret == True:
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
    
            # Find the rotation and translation vectors.
            rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)
            
            # # Specific to particular chessboard used in example
            # # project 3D points to image plane
            # imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
    """
    

# ## MISC DISPLAY FUNCTIONS ##

# def draw(img, corners, imgpts):
#     corner = tuple(corners[0].ravel())
#     img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
#     img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
#     img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
#     return img

# # Display barcode and QR code location
# def display(im,decodedObjects):
#     hull=pull_convex(decodedObjects)
    
#     # Number of points in the convex hull
#     n = len(hull)

#     # Draw the convext hull
#     for j in range(0,n):
#       cv2.line(im, hull[j], hull[ (j+1) % n], (255,0,0), 3)

#   # Display results
#   cv2.imshow("Results", im);
#   cv2.waitKey(0);

"""
# Fix this snippet

            img = draw(img,corners2,imgpts)
            cv2.imshow('img',img)
            k = cv2.waitKey(0) & 0xff
            if k == 's':
                cv2.imwrite(fname[:6]+'.png', img)
    
    cv2.destroyAllWindows()
"""

# Main
if __name__ == '__main__':

  # Read image
  im = cv2.imread(r'example_img\zbar-test.jpg')
  
  decodedObjects=decode(im)
  rvecs,tvecs=est_pose(im,decodedObjects) 
  #02SEP21 ERROR, est_pose:
  #TypeError: cannot unpack non-iterable NoneType object
