# -*- coding: utf-8 -*-
"""
REV0: 16MAR22 by Alec Engl
# library containing all functions pertinent to RSAT QR pose est
# REFS:
    https://github.com/KyleJosling/QR-Pose-Position (Classical QR detecition)
    https://github.com/niconielsen32/ComputerVision.git (Camera Calibration)
    pyimagesource
    learnOpenCV

@author: Alec
"""

import cv2
import math
import numpy as np
import glob
import os

#################################### QR DETECTION ####################################

#These functions search for and return the points in the QR code that are used
#to find the pose and position of the camera.

class Detector:
    def __init__(self):
        print("I'm alive!")

#This function gets two points from each corner square
def getSquarePoints(points,center):

	#Search for max and min distances, return the indices
	maxIndex=0
	maxVal=0
	minIndex=0
	minVal=distance(points[0][0],center)
	for i in range(0,4):
		cValue=distance(points[i][0],center)
		if cValue > maxVal:
			maxVal=cValue
			maxIndex=i
		if cValue<minVal:
			minVal=cValue
			minIndex=i
	return points[maxIndex][0],points[minIndex][0]


#Gets distance between two points
def distance(p,q):
	return math.sqrt(math.pow(math.fabs(p[0]-q[0]),2)+math.pow(math.fabs(p[1]-q[1]),2))

#Gets the height of the triangle formed by all three squares using some trig
#Can return a negative height, this is how we find the orientation of the corners
def triangleHeight(l,m,j):
	a = -((m[1] - l[1])/(m[0] - l[0]))
	b = 1.0
	c = (((m[1] - l[1])/(m[0] - l[0]))*l[0]) - l[1]
	try:
		pdist = (a*j[0]+(b*j[1])+c)/math.sqrt((a*a)+(b*b))
	except:
		return 0
	else:
		return pdist

#Gets line slope between two points
def slope(l,m):
	dx=m[0]-l[0]
	dy=m[1]-l[1]

	dxy=float(dy)/float(dx)
	return round(dxy)

#Finds the orientation of the corner squares by comparing distances between the centers
def findCornerOr(centers):
	AB=distance(centers[0],centers[1])
	BC=distance(centers[1],centers[2])
	AC=distance(centers[0],centers[2])

	#Find the outlier corner by comparing distances
	if(AB>BC and AB>AC):
		outlier = 2
		median1 = 0
		median2 = 1
	elif(AC>AB and AC>BC):
		outlier = 1
		median1 = 0
		median2 = 2
	elif(BC>AB and BC>AC):
		outlier = 0
		median1 = 1
		median2 = 2

	#Now find which corner is left, which one is right
	#Certain conditions must be met to determine the orientation of the QR code
	#We determine this using slopes between the points and height of triangle
	#formed by the corner squares
	ang=slope(centers[median1],centers[median2])
	ang2=slope(centers[median1],centers[outlier])
	height=triangleHeight(centers[median1],centers[median2],centers[outlier])

	if(ang < 0 and height < 0):
		bottom = median1
		right = median2
	elif(ang > 0 and height < 0 and ang2!=0):
		right = median1
		bottom = median2
	elif (ang>0 and height < 0 and ang2==0):
		right=median2
		bottom=median1
	elif(ang < 0 and height > 0):
		right = median1
		bottom = median2
	elif(ang > 0 and height > 0):
		bottom = median1
		right = median2
	elif ang==0:
		bottom=median1
		right=median2
		
	#Return the index of the outlier, bottom and right square
	return outlier,bottom,right

#Function returns all the points found in each of the three corner squares
#within the QRCode
def getPoints(imagePath_or_image,G1=3,G2=5,debug=False):## **MAY WANT TO MODIFY DEFAULT COVARIANCE VALUES
    varType=type(imagePath_or_image)
	#We will return this array of points later
    returnArray=np.zeros((6,2),dtype=np.float32)
    
    assert (varType==str) or (varType==np.ndarray) # Ensures that variable is either a path string or an image array
    
    if type(imagePath_or_image)==str:
           #Load the image
           origImg=cv2.imread(imagePath_or_image)
    else:
        origImg = imagePath_or_image
        
	#set lower and upper bounds to filter out background
    ## **MAY WANT TO CHANGE THIS
    lower=np.array([180,180,180],dtype='uint8')
    upper=np.array([255,255,255],dtype='uint8')

	#Remove anything that isnt white from image
    mask=cv2.inRange(origImg,lower,upper)
    img=cv2.bitwise_and(origImg,origImg,mask=mask)

    gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

		#Gaussian blur to find edges
    img=cv2.GaussianBlur(gray,tuple((G1,G2)),0)
    if debug:
        cv2.namedWindow('blur',cv2.WINDOW_NORMAL)
        cv2.imshow('blur',img)
        cv2.resizeWindow('blur',600,600)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

	#Find edges using canny
    edges=cv2.Canny(img,10,400)
    if debug:
        cv2.namedWindow('Canny',cv2.WINDOW_NORMAL)
        cv2.imshow('Canny',edges)
        cv2.resizeWindow('Canny',600,600)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

	#Get contours,hierarchy
    contours,hierarchy=cv2.findContours(edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) ## * _SIMPLE saves only the endpoints of lines, not every point on them (_NONE does that)
    
    if debug:
        # cont = cv2.drawContours(edges, contours, -1, (255,255,255), 3)
        cont = cv2.drawContours(origImg, contours, 0, (255,0,0), 7)
        cv2.namedWindow('cont',cv2.WINDOW_NORMAL)
        cv2.imshow('cont',cont)
        cv2.resizeWindow('cont',600,600)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    bContourArea=cv2.contourArea(contours[0]) # Save area of the largest contour
    
    if bContourArea == 0: # If no contour detected
        return returnArray,0    

 	#Compute the center of the big contour for later
    M=cv2.moments(contours[0]) # Wouldn't m00 be the same as bCOntourArea
    cX=int(M["m10"]/M["m00"])
    cY=int(M["m01"]/M["m00"])

    patternCenter=tuple((cX,cY))

 	#This list stores the corner contours
    cornerContours=[]

 	#Get the area of the contours and get the contours that define the corner squares
    for i in range(0,len(contours)):
        area=cv2.contourArea(contours[i])

		#**TODO - make this different
        if area:
            ratio=bContourArea/area
        else:
            ratio=0

		#Found a corner square if ratio between area of paper contour and square
		#lies in this range.
        if (ratio > 90) and (ratio<115):
            cornerContours.append(i)


    centers=[]
    squares=[]
 	#We only need every other contour because two are returned for each corner square**
    for i in cornerContours[::2]:
		#Approximate the shape of the contour to get the square points
        epsilon=0.1*cv2.arcLength(contours[i],True)
        approx=cv2.approxPolyDP(contours[i],epsilon,True)
		#Average x and y points of each box to find center of each corner square
        x=0
        y=0
        for k in range(0,4):
            x+=approx[k][0][0]
            y+=approx[k][0][1]
        centers.append(tuple((x/4,y/4))) # Computes the average of the 4 previously summed points
        squares.append(approx) # Adds whole square

    if len(squares)<3:
        return returnArray,0

    #Find the orientation of the corner squares
    outlier,bottom,right=findCornerOr(centers)

 	#We build an array of points to return :
 	#Outlier outer,outlier inner, bottom outer, bottom inner, right outer, right inner
    returnArray[0],returnArray[1]=getSquarePoints(squares[outlier],patternCenter)
    returnArray[2],returnArray[3]=getSquarePoints(squares[bottom],patternCenter)
    returnArray[4],returnArray[5]=getSquarePoints(squares[right],patternCenter)
    returnArray=np.expand_dims(returnArray,axis=1)
    
    return returnArray,1
    # return returnArray,0
    
# Call one of several candidate detectors
def testDetector(detector,useCamStream = False, path = 'easyImages_Engl\\rollOnly'):
    ## ** implement some sort of calibration/distortion check here specific to camera
    # cv2.imshow("Image taken",img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    # dst2 = undistort(img,mtx,newmtx,dist)
    # print("Image undistorted")
    # img_variants = [img,dst2]
    # img_str = ['Distorted', 'Undistorted']
    
    ## Pre-loop config
    if useCamStream: # Livestream
        ##* pass camera index as input var
        cam=cv2.VideoCapture(1) #Seems to alternately choose front or back webcam on surface
    else: # Pull from directory
        ##* pass input directory as input var
        #Load the images
        # images=glob.glob('markerImages/*.JPG')
        # images=glob.glob('markerImages_Engl/*.JPG')
        images=glob.glob(os.path.join(path,'*.png'))
    
    ## Loop over all test images/livestream webcam
    i = 0 # Number of frames processed
    while(True):
        
        ## Grab image/frame
        if useCamStream: # Livestream
            ret, img=cam.read()
            print("Image taken")
        else: # Pull from directory
            fname = images[i]
            print("File name: " + str(fname))
            img=cv2.imread(fname)
            
        imagePoints,success=getPoints(fname)
        if not success:
            imagePoints,success=getPoints(fname,7,5)
        
        
            data, bbox, _ = detector.detectAndDecode(image)
            
            if(bbox is not None):
                #     for i in range(len(bbox)):
                #         cv2.line(image, (int(bbox[i][0][0]),int(bbox[i][0][1])), (int(bbox[(i+1) % len(bbox)][0][0]),int(bbox[(i+1) % len(bbox)][0][1])), color=(255,
                #                   0, 0), thickness=2)
                #     cv2.putText(image, data, (int(bbox[0][0][0]), int(bbox[0][0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX,
                #                 1, (255, 250, 120), 2)
            
        ## Either way, whether detected or not, display image
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
    if useCamStream:
        cam.release()
    cv2.destroyAllWindows()

## Custom classical QR detector
def classQRdetect(img):
    
    
#################################### QR POSE ESTIMATION ####################################    
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
    
#################################### CAM CALIBRATION ####################################

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