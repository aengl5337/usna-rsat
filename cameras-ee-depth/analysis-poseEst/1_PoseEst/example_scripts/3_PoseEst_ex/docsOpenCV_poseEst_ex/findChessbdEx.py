# -*- coding: utf-8 -*-
"""
Created on Tue Apr  6 17:17:28 2021

Pulled from openCV tutorial: https://github.com/abidrahmank/OpenCV2-Python-Tutorials/blob/master/source/py_tutorials/py_calib3d/py_pose/py_pose.rst

@author: m211740
"""

import cv2
import numpy as np
import glob
import time

# Load previously saved data, camera matrix and distortion coefficients from the previous calibration result.
# See py_calibration.py in adjacent directory

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

fileOpt=input("File (1) or webcam shot(0)?\n")
chessbdDimsR=int(input("Rows of chessboard:\n"))
chessbdDimsC=int(input("Columns of chessboard:\n"))

chessbdDims=(chessbdDimsR-1,chessbdDimsC-1)

if fileOpt==1:
    fname=r'C:\Users\Alec\Documents\NSTAR\Software\Python\Depth and Pose Estimation - ODROID+D435\PoseEst\QR_PoseEst\docsOpenCV_poseEst_ex\8x8_ex.jpg'
    img = cv2.imread(fname)
else:
    print("Prepare chessboard in webcam frame!")
    input("Press enter when ready:")
    cam=cv2.VideoCapture(0)
    retCam,img=cam.read()

gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
# NOTE: input 'patternSize' is not the grid dimensions of the board, but rather
# the number of internal points (i.e. =(grid dims-1))
ret, corners = cv2.findChessboardCorners(gray, chessbdDims, None)
# For a [2-D] MxN chessboard, returns a numpy.ndarray of corners with dims ((M-1)*(N-1),1,2)
"""
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

Corners are formatted (x,y), i.e. (cols,rows), and iterated from left to right, top to bottom (starting in the northwest-most corner)
"""

for each in corners:
    each=each[0]
    print(each)
    
    gray= cv2.circle(gray, (int(each[0]),int(each[1])), radius=10, color=(0, 0, 255), thickness=-1) # Note that this truncates the result
    cv2.imshow("Results", gray)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
# Numbers are to subPix precision, but need some refinement
if ret == True:
    corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
    # Keeps same dims as input corners
    # May seem completely unnecessary, as tests show there is no difference between in and output here
    # However, this is because the input variable is modified through the function
    