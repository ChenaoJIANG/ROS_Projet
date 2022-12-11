#!/usr/bin/env python3

# This is the node for challenge1.1, 1.2 and 3 (line following, obstacle avoidance and navigation in a cluttered environment)
# This part of the radar and camera cooperation. Avoid obstacles based on radar data when there are obstacles,
# and follow the line based on image data when there are no obstacles

# Import the required packages, functions and modules
import rospy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

import numpy as np
import cv2
import cv_bridge
from fonction import laser_line,laser_obstacle,laser_env,mask_line,mask_obstacle

#attribut of bridge type
cvBridge = cv_bridge.CvBridge()
 
global obstacle # globle globle boolean, True = there is obstacle
obstacle = True
global turn_left # globle variable, -1: go foward; 0: turn right; 1: turn left


# callback function for the message of LaserScan 
def callback_laser(msg):
    # global variables for different challenges
    global obstacle
    global turn_left
    global line
    global line_obstacle
    global env
    if line:   #if the robot is in the direct line (following line)
        print("Challenge1_1: Line following")
        front,front_l,front_r=laser_line(msg)  #laser detection of the front and right line to pursue the path 
        dis = 0.3   # the limit of distence
        dis_l = 0.2   # the limit of distence for front_l
        dis_r = 0.2   # the limit of distence for front_r

    elif line_obstacle: #if the robot is facing an obstacle
        print("Challenge1_2: Line + obstacle avoidance")
        front,front_l,front_r=laser_obstacle(msg) #laser detection of the obtacles
        dis = 0.25  # distance for front
        dis_l = 0.22   # the limit of distence for front_l (left)
        dis_r = 0.27   # the limit of distence for front_r (right)

    elif env: #if the robot is in the in the cluttered environment
        print("Challenge3: Navigation in a cluttered environment")
        front,front_l,front_r=laser_env(msg) #laser detection of the cluttered environment
        dis = 0.25   # the limit of distance
        dis_l = 0.23  # the limit of distance for front_l
        dis_r = 0.23   # the limit of distance for front_r


    if line or line_obstacle or env: #if the robot is in one of the previous cases
        #print("laser works!") 
        if front<dis or front_l<dis_l or front_r<dis_r: #if each distance is lower than its limit distance 
            obstacle = True
            #print("there is obstacle!")
            if (front>dis and front_l<dis_l and front_r<dis_r): #if the distance to an obstacle in front is bigger than the limit one and the other distances are smaller than their limit ones (the doesn't face an obstcale in fornt but in right front and in left front)
                # go forward
                turn_left = -1
            elif (front<dis and front_l>dis_l and front_r<dis_r) or (front>dis and front_l>dis_l and front_r<dis_r): #if the robot is facing an obstacle except in front left
                # turn left
                turn_left = 1

            elif (front>dis and front_l<dis_l and front_r>dis_r) or (front<dis and front_l<dis_l and front_r>dis_r) or (front<dis and front_l>dis_l and front_r>dis_r): #if the robot is facing an obstacle except in front right
                # turn right
                turn_left = 0       
            elif front<dis and front_l<dis_l and front_r<dis_r: #the distances are smaller than their limit ones
                if front_l>front_r:
                    # turn left
                    turn_left = 1
                else:
                    # turn right
                    turn_left = 0
        else:  # there is no obstacle
            obstacle = False
            #print("no obstacle ")


# callback function for the message of Image
def callback_image2cv(msg):
    global obstacle 
    global line      # Mainly: follow line; Sencondary: avoid obstacle
    global line_obstacle  # Mainly: avoid obstacle; Sencondary: follow line
    global env      # last challenge
    
    # Transform the image to openCV format, msg is the Image from sensor_msgs
    cvImage = cvBridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    rows,cols,channals=cvImage.shape

    if line_obstacle: # Challenge 1_2, Get mask by the function maske_obstacle
         mask,mask_y,mask_w = mask_obstacle(cvImage)
    else:
        mask,mask_y,mask_w = mask_line(cvImage)   # Challenge 1_1 and 3, get les same masks by the function mask_line
    
    # merging the two images (masks)
    image_rest = cv2.bitwise_and(cvImage,cvImage,mask=mask)

    # Compute the mask moments
    M_left = cv2.moments(mask_y)
    M_right = cv2.moments(mask_w)
    
    if M_left["m00"]>0:
        # calculer x,y center of the left line
        cX_left = int(M_left["m10"] / M_left["m00"])
        cY_left = int(M_left["m01"] / M_left["m00"])
    
    else:
        # if there is no line on left, set the center at (5,180)
        cX_left = 5
        cY_left = 180 #rows*3/4

    if M_right["m00"]>0:
        # calculer x,y center of the right line
        cX_right = int(M_right["m10"] / M_right["m00"])
        cY_right = int(M_right["m01"] / M_right["m00"])
        
    else:
        # if there is no line on rignt, set the center at (315,180)
        cX_right = 315  #cols -5
        cY_right = 180  #rows*3/4

    # Calculate the midpoint of the center of the two lines as the target point
    cX = int((cX_left+cX_right)/2)
    cY = int((cY_left+cY_right)/2)

   
    # show thest informations in image
    cv2.circle(image_rest,(cX_left,cY_left),2,(255,0,0),2)   # center of yellow line
    cv2.circle(image_rest,(cX_right,cY_right),2,(255,0,0),2)  # center of white line
    cv2.circle(image_rest,(cX,cY),3,(0,255,0),2)     # center of two lines( goal point)
    cv2.circle(image_rest,(int(cols/2),cY),2,(0,0,255),2)    # center of robot (center of image)
	
    #the mask showing
    cv2.imshow('image rest',image_rest)  #image displaying (showing)
    cv2.waitKey(3) #showing the image 3seconds before it automatically closes


    #velocity commands for robot command
    vel_msg = Twist() #attribut of Twist type
    if line or line_obstacle or env:  #if the robot is passing a simple path with line following or facing an obstacle or it is inside a cluttered environment 
        if obstacle:  #if there's an obstacle
            Trun(vel_msg)    # Obstacle avoidance function Trun works

        #s'il n'y a pas d'obstacle, on suit la ligne
        else:
            errorX = cX - int(cols/2) #error calculation
            linear_scale = 0.25
            angular_scale = 1/30
            slow = 1/2  

            if abs(errorX)>25:
                vel_msg.linear.x = linear_scale*slow   # slow down when making a big turn
            else:
                vel_msg.linear.x = linear_scale #regular pass of the robot

            # because errorX has a sign, there is no need to divide the right and left cases
            vel_msg.angular.z = (-1)* angular_scale * errorX #angular speed command based on the error
            #print("camera works")
            #rospy.loginfo(vel_msg)

            vel_pub.publish(vel_msg)
    else:
        #print("Not line following")
        pass
        

# This is a function of the speed command: parameters and paths
def Trun(vel_msg):   
    
    #getting the parameters: linear and angular respectively
    linear_scale1 = rospy.get_param("linear_scale1")
    angular_scale1 = rospy.get_param("angular_scale1")

    if turn_left == -1:  # go forward
        vel_msg.linear.x = linear_scale1
        #print("go forward! ")
        #rospy.loginfo(vel_msg)
        vel_pub.publish(vel_msg)

    elif turn_left == 0: # turn right
        vel_msg.linear.x = 0.01
        vel_msg.angular.z = (-1)*angular_scale1
        #print("turn right! ")
        #rospy.loginfo(vel_msg)
        vel_pub.publish(vel_msg)

    elif turn_left== 1: # turn left
        vel_msg.linear.x = 0.01
        vel_msg.angular.z = angular_scale1
        #print("turn left! ")
        #rospy.loginfo(vel_msg)
        vel_pub.publish(vel_msg)

    elif turn_left == 5:  #robot stop
    	#the two velocities are zero 
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        #print("stop! ")
        #rospy.loginfo(vel_msg)
        vel_pub.publish(vel_msg)


#  callback function for the message of Int16(nb_red)
def callback_red(msg):
    global line
    global line_obstacle
    global env
    if msg.data==6:   # couloir
        line = False
        line_obstacle = False
        env = False
    elif msg.data==4 or msg.data==5:  # line + obstacle
        line = False
        line_obstacle = True
        env = False
    elif msg.data==10:   # environement
        line = False
        line_obstacle = False
        env = True
    else:  # line
        line = True
        line_obstacle = False
        env = False



   
if __name__=='__main__':
    
   
    try:
        global line
        line = True
        global line_obstacle
        global env
        # build a new node named get_image
        rospy.init_node('challenge1and3',anonymous=True)
        
	    #create a publiser to publise the topic 'cmd_vel',the type of message is Twist and the length of queue is 10
        vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # create a subscriber receive the topic /connect, the type of msg is Int16 and then give to the callback_red function
        rospy.Subscriber('/connect',Int16, callback_red)

        #create a subscriber receive the topic /scan, the type of msg is LaserScan and then give to the callback_laser function
        scna_sub = rospy.Subscriber("/scan",LaserScan,callback_laser)
   	
   	    #create a subscriber receive the topic /camera/image, the type of msg is Image and then give to the callback_image2cv function
        image_sub=rospy.Subscriber("/camera/image",Image, callback_image2cv)

        # keeps the node from exiting until the node has been shutdown
        rospy.spin()

    except rospy.ROSInterruptException:
        print('shutting down')


