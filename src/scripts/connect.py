#!/usr/bin/env python3

# This is a node for connecting different challenges, named contact
# It first subscribes to the information of the camera, obtains the number of red lines through image processing
# and then publishes a topic named /contact, which contains the information of the number of red lines


# Import the required packages and modules
import rospy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

import numpy as np
import cv2
import cv_bridge

   
cvBridge = cv_bridge.CvBridge()   # cvBridge provides the required functions to convert the image from ROS to OpenCV
 
# Set les global variables
global nb_red    # Record the number of times the red line appears
nb_red =0        # initial value is 0

global red_yes, red_no   # Record if the red line appears
red_yes=False   
red_no=False


# Callback function for image message 
def callback(msg):  # The class of msg is Image

    # Transform the image to openCV format, msg is the Image from sensor_msgs
    cvImage = cvBridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # Change color represtation from BGR to HSV
    hsv = cv2.cvtColor(cvImage,cv2.COLOR_BGR2HSV)

    #lower and upper boundary for red
    lower_red = np.array([156,43,46])
    upper_red = np.array([180,255,255])

    # image binarisation
    mask_r = cv2.inRange(hsv,lower_red,upper_red)
    
    # limit mask, the red line only needs to detect a small part
    rows,cols,channals=cvImage.shape
    mask_r[0:int(rows*6/7),:]=0
    mask_r[:,0:int(cols*3/7)]=0
    mask_r[:,int(cols*5/7):]=0

  
    # ----------nb red line-----------
    global nb_red
    global red_yes,red_no
    M_red = cv2.moments(mask_r)   # Compute the mask moment

    # Record the number of red lines by their appearance and disappearance
    if M_red["m00"]>0:     # If the red line area is greater than 0, it appears
        red_yes = True     
    else:                  # If the red line area less than or equal to 0, it disappears
        red_no = True

    # If the red line appears and then disappears or disappears and then reappears, add one to the number
    if red_yes==True and red_no==True:   
        nb_red += 1
        red_yes = False
        red_no = False
        # So when the robot passes a red line, the red line will go from disappearing 
        # to appearing and then disappearing again, and the number will increase twice

    if nb_red>=12:  # passed a circle, return zero
        nb_red = 0
    
    nb_pub.publish(nb_red)  # Publish the number of red lines through the topic /connect


        
   
if __name__=='__main__':
    
   
    try:
        # built a new node named get_image
        rospy.init_node('connect',anonymous=True)
        # creat a subscriber receive the topic /camera/image Image message then give to the callback function
        image_sub=rospy.Subscriber("/camera/image",Image, callback)
        # creat a publisher to publise the topic /connect, the type of message is Int16
        nb_pub = rospy.Publisher('/connect', Int16, queue_size=10)

        r = rospy.Rate(10)   # looping at the desired rate 10Hz
        rospy.spin()      # keeps the node from exiting until the node has been shutdown
        

    except rospy.ROSInterruptException:
        print('shutting down')


