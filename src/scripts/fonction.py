#!/usr/bin/env python3
# his file contains functions for radar and camera data processing. 
# The general idea is the same, but the parameters are optimized according to different challenges.

# Import the required modules
import numpy as np
import cv2
import cv_bridge

cvBridge = cv_bridge.CvBridge() # cvBridge provides the required functions to convert the image from ROS to OpenCV
 
# ======== 3 functions for radar data processing ============
# -----------------------------------------------------------------------
# 1. Processing radar data for the line-following section
# This part of the radar has little effect, mainly to avoid obstacles ahead to prevent accidents
def laser_line(msg):  # msg is radar data LaserScan
        limit = 10   # laser range in each direction
        
        # Calculate the minimum value in the range as the distance from the obstacle in that direction
        front = (np.min(msg.ranges[0:limit])+np.min(msg.ranges[360-limit:360]))  # The front radar range is 350-360 and 0-10
        front_l = np.min(msg.ranges[20-limit:20+limit])  # The front left radar range is 10-30
        front_r = np.min(msg.ranges[340-limit:340+limit])  # The front right radar range is 330-350

        return front,front_l,front_r   # Returns distances in three directions for subsequent operations


# ------------------------------------------------------------------------
# 2. Processing radar data for the line-following plus obstacle avoidance section
# This part of the radar is very important, requires a larger range to effectively avoid obstacles
def laser_obstacle(msg):
        limit = 20   # front range limit
        limit1 = 15   # front left and front right range limit


        front = (np.min(msg.ranges[0:limit])+np.min(msg.ranges[360-limit:360]))/2
        left = np.min(msg.ranges[90-limit:90+limit])
        right = np.min(msg.ranges[270-limit:270+limit])
        back = np.min(msg.ranges[180-limit:180+limit])

        front_l = np.min(msg.ranges[40-limit1:40+limit1])
        front_r = np.min(msg.ranges[320-limit1:320+limit1])

        return front,front_l,front_r

# --------------------------------------------------------------------------
# 3. Processing radar data for the navigation in a cluttered environment
# This part of the radar is also important
def laser_env(msg):
        limit = 10   
        limit1 = 15

        front = (np.min(msg.ranges[0:limit])+np.min(msg.ranges[360-limit:360]))/2
        left = np.min(msg.ranges[90-limit:90+limit])
        right = np.min(msg.ranges[270-limit:270+limit])
        back = np.min(msg.ranges[180-limit:180+limit])

        front_l = np.min(msg.ranges[30-limit1:30+limit1])
        front_r = np.min(msg.ranges[330-limit1:330+limit1])

        return front,front_l,front_r


# ================ 2 functions for camera data processing =================================== 

# 1. Processing camera data for the line following and the navigation in a cluttered environment
# This part needs to limit the mask to be smaller in order to avoid the interference of surrounding lines
def mask_line(cvImage):   # cvImage is the OpenCV class

    # Change color represtation from BGR to HSV
    hsv = cv2.cvtColor(cvImage,cv2.COLOR_BGR2HSV)

    #lower and upper boundary for white
    lower_white = np.array([0,0,215])
    upper_white = np.array([0,40,255])

    #lower and upper boundary for red
    lower_red = np.array([156,43,46])
    upper_red = np.array([180,255,255])

    #lower and upper boundary for yellow
    lower_yellow = np.array([26,43,46])
    upper_yellow = np.array([34,255,255])

    # image binarisation, different masks for different colors
    mask_y = cv2.inRange(hsv,lower_yellow,upper_yellow)
    mask_w = cv2.inRange(hsv,lower_white, upper_white)
    mask_r = cv2.inRange(hsv,lower_red,upper_red)
    
    # limit mask
    # Make the unneeded part black because the lines far and beside it will affect the extent of the line
    
    rows,cols,channals=cvImage.shape  # get the image's information 

    mask_y[0:int(rows*2/3),:]=0     # Mask off the top 2/3 of the image
    mask_y[:,0:int(cols/8)]=0       # Mask off the left 1/8 of the 
    mask_y[:,int(cols*2/3):]=0      # Mask off the right 1/3 of the  

    mask_w[0:int(rows*3/4),:]=0     # Mask off the top 3/4 of the image, a little different from the yellow mask, it can pass the turntable better
    mask_w[:,0:int(cols/3)]=0       # Mask off the left 1/3 of the 

    # mask for the red line, the red line only needs to detect a small part
    mask_r[0:int(rows*6/7),:]=0  
    mask_r[:,0:int(cols*3/7)]=0
    mask_r[:,int(cols*5/7):]=0
    
    # combine two masks
    mask = cv2.bitwise_or(mask_y,mask_w)
    mask = cv2.bitwise_or(mask,mask_r)

    return mask,mask_y,mask_w   # return les masks for next operation


# 2. Processing camera data for the line following plus obstacle avoidance
# This part is mainly about obstacle avoidance, the mask should be a little bigger than other challenges
def mask_obstacle(cvImage):

    # Change color represtation from BGR to HSV
    hsv = cv2.cvtColor(cvImage,cv2.COLOR_BGR2HSV)

    #lower and upper boundary for white
    lower_white = np.array([0,0,215])
    upper_white = np.array([0,40,255])

    #lower and upper boundary for red
    lower_red = np.array([156,43,46])
    upper_red = np.array([180,255,255])

    #lower and upper boundary for yellow
    lower_yellow = np.array([26,43,46])
    upper_yellow = np.array([34,255,255])

    # image binarisation
    mask_y = cv2.inRange(hsv,lower_yellow,upper_yellow)
    mask_w = cv2.inRange(hsv,lower_white, upper_white)
    mask_r = cv2.inRange(hsv,lower_red,upper_red)
    
    # limit mask
    rows,cols,channals=cvImage.shape

    mask_y[0:int(rows*2/3),:]=0     # Mask off the top 2/3 of the image
    mask_y[:,int(cols*2/3):]=0      # Mask off the right 2/3 of the image

    mask_w[0:int(rows*3/4),:]=0     # Mask off the top 3/4 of the image
    mask_w[:,0:int(cols/3)]=0       # Mask off the left 2/3 of the image

    mask_r[0:int(rows*6/7),:]=0
    mask_r[:,0:int(cols*3/7)]=0
    mask_r[:,int(cols*5/7):]=0
    
    # combine two masks
    mask = cv2.bitwise_or(mask_y,mask_w)
    mask = cv2.bitwise_or(mask,mask_r)

    return mask,mask_y,mask_w
