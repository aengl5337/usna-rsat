"""
saveDepthCamData.py

See detect_distance.py (PySource, https://www.youtube.com/watch?v=mFLZkdH1yLE)

This script initializes and pulls data from a connected realsense camera
It displays the depth of the point over which the mouse hovers in the RGB feed


"""

# Library imports
import cv2
import pyrealsense2 as rs

# .py imports
from realsense_depth import *

# Setup function for mouse event
# def show_distance(event, x, y, args, params):
#     global point # This variable must be global to 
#     point = (x, y)


# Initialize point in frame, then update point realtime based on mouse location
point=(400,300)

# Initialize Depth Camera
dc = DepthCamera()

ret, depth_frame, color_frame = dc.get_frame()

# # Create mouse event
# cv2.namedWindow("Color Frame")
# cv2.setMouseCallback("Color Frame", show_distance)

# while True:

#     ret, depth_frame, color_frame = dc.get_frame()

#     depth = depth_frame[point[1], point[0]]
#     cv2.circle(color_frame, point, 4, (0, 0, 255))
#     cv2.putText(color_frame,"%dmm" % depth, (point[0],point[1]-20), cv2.FONT_HERSHEY_PLAIN, 2, (0,0,255), 2)

#     cv2.imshow("Color Frame", color_frame)

#     key = cv2.waitKey(1)    # Waits 1 ms, breaks from loop if 'esc' pressed

#     if key == 27:
#         break
    
# cv2.destroyAllWindows()

