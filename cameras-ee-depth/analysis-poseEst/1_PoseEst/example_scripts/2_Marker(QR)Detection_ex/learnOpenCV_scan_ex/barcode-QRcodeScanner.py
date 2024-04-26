# -*- coding: utf-8 -*-
"""
Created on Tue Apr  6 16:53:47 2021

Pulled from learnOpenCV's https://github.com/spmallick/learnopencv/blob/master/barcode-QRcodeScanner/barcode-QRcodeScanner.py

Instruction article here: https://learnopencv.com/barcode-and-qr-code-scanner-using-zbar-and-opencv/

@author: m211740
"""

from __future__ import print_function
import pyzbar.pyzbar as pyzbar
import numpy as np
import cv2

def decode(im) :
  # Find barcodes and QR codes
  decodedObjects = pyzbar.decode(im)

  # Print results
  for obj in decodedObjects:
    print('Type : ', obj.type)
    print('Data : ', str(obj.data),'\n')

  return decodedObjects


# Display barcode and QR code location
def display(im, decodedObjects):

  # Loop over all decoded objects
  for decodedObject in decodedObjects:
    points = decodedObject.polygon

    # If the points do not form a quad, find convex hull
    if len(points) > 4 :
      hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
      hull = list(map(tuple, np.squeeze(hull)))
    else :
      hull = points;

    # Number of points in 0the convex hull
    n = len(hull)

    # Draw the convext hull
    for j in range(0,n):
      # Draws line from 0 to 1, 1 to 2, 2 to 3, and 3 to 0 (modulo kicks in)
      # cv2.line(im, hull[j], hull[ (j+1) % n], (255,0,0), 3)
      im= cv2.circle(im, hull[j], radius=20, color=(0, 0, 255), thickness=-1)
      cv2.imshow("Results %d"%(j+1), im);
      cv2.waitKey(0);
      cv2.destroyAllWindows()
      

  # Display results
  
  return hull


# Main
if __name__ == '__main__':

  # Read image
  im = cv2.imread('zbar-test.jpg')

  decodedObjects = decode(im)
  hull=display(im, decodedObjects)