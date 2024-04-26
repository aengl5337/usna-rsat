# -*- coding: utf-8 -*-
"""
Created on Tue Aug  8 16:39:10 2023

hdSaveImagesExample.py

()-Debug after adding main()
()-(not important)Fix cv2.waitKey won't toggle on 'ESC' key unless cv2 displays an image
()-what are supported pickle file endings?
()-Fix dt~0 on first frame (initialization too fast?)
()+Add different compression alg fcnality in safe_open_w    
    with open('no_compression.pickle', 'wb') as f:
        pickle.dump(data, f)
    
    with gzip.open("gzip_test.gz", "wb") as f:
        pickle.dump(data, f)
    
    with bz2.BZ2File('bz2_test.pbz2', 'wb') as f:
        pickle.dump(data, f)
    
    with lzma.open("lzma_test.xz", "wb") as f:
        pickle.dump(data, f)
    
    with open('no_compression.pickle', 'rb') as f:
        pdata = f.read()
        with open('brotli_test.bt', 'wb') as b:
            b.write(brotli.compress(pdata))

@author: Alec
"""

# Library imports
import cv2
import pyrealsense2 as rs
import time
import os, os.path
import errno
import numpy as np
import matplotlib.pyplot as plt
import bz2
import gzip
import lzma
import brotli
import pickle

# .py imports
from realsense_depth import *

def mkdir_p(path):
    """
    Implements mkdir -p (make all parent directories if don't exist)
    Taken from https://stackoverflow.com/a/600612/119527
    
    INPUTS
    
    OUTPUTS                     
    
    """
    try:
        os.makedirs(path)
    except OSError as exc: # Python >2.5
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else: raise

def safe_open_w(path):
    """
    Open "path" (regardless of whether it existed previously) for writing, creating any parent directories as needed.
    Taken from https://stackoverflow.com/a/600612/119527
    
    Parameters
    ----------
    path : TYPE
        Current directory (can also include filename).

    Returns
    -------
    TYPE
        DESCRIPTION.

    """
    mkdir_p(os.path.dirname(path))
    # return open(path, 'w')
    return open(path,'wb')

def SAVE(data,path):    
    """
    
    
    Parameters
    ----------
    data : TYPE
        DESCRIPTION.
    fpath : TYPE
        Current directory plus filename, with file ending

    Returns
    -------
    None.

    """
    # with open(fname,'wb') as file:
    fname = os.path.basename(os.path.normpath(path)) # Normpath normalizes the path
    print("**SAVING %s**"%fname)
    with safe_open_w(path) as file:
        pickle.dump(data, file, pickle.HIGHEST_PROTOCOL)
    print("**SAVED %s**"%fname)
    
##########################################################################
def main():
    # Save path
    spath = "E:\\NSTAR\\Software\\Python\\Depth and Pose Estimation - ODROID+D435\\2_DepthCam\\hdsave"
    SAVEF = True
    
    # Initialize Depth Camera
    dc = DepthCamera()
    
    ret, depth_frame, color_frame = dc.get_frame()
    i=0
    DT = 0
    t0 = time.time() # Units are seconds
    while DT<10: # for 10s
        ret, depth_frame, color_frame = dc.get_frame()
        
        if SAVEF:
            fname1 = "depthframe_%d.pkl" %i
            fname2 = "rgbframe_%d.pkl" %i
            fpath1 = os.path.join(spath,fname1)
            fpath2 = os.path.join(spath,fname2)
            SAVE(depth_frame,fpath1)
            SAVE(color_frame,fpath2)
        
        # key = cv2.waitKey(1)    # Waits 1 ms, breaks from loop if 'esc' pressed
        # if key == 27:
        #     break
        
        # Evaluate timing
        t1 = time.time()
        dt = t1-t0
        DT += dt
        FPS = 1/dt
        print("%d; iFPS=%.3f with saving=%r"%(i,FPS,SAVEF))
        
        # Reset
        t0 = t1
        i+=1
    
    print("Total FPS=%f"%((i+1)/DT))
    
    # cv2.destroyAllWindows()
    
if __name__ == "__main__":
    main()
    