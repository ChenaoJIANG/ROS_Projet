#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# This is the node for challenge2 (movement in a corridor)
# Only the radar data is needed to move the robot according to the distance in front, front left and front right

# Import the required packages and modules
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16

# callback function for the message of LaserScan 
def callback(msg):
    global couloir   # global bool
    # if the robot reaches the corridor(number of red line is 6), processing radar data
    if couloir==True:
        print("Challenge2: Couloir")
        vel_msg = Twist()  # a message type of Twist
	
	# distance control thresholds
        limit = 15   # front range limit
        limit1 = 10   # front left and right range limit
        dis = 0.24    # distance to left or right front obstacle
        dis1 = 0.25   # distance to front obstacle
	
	# Construction of vectors containing the LDS measures of the angles (stocked in the following variables)
        front = (np.min(msg.ranges[0:limit])+np.min(msg.ranges[360-limit:360]))/2 #Variable characterizing the distance between the robot and the obstacles in front
        left = np.min(msg.ranges[90-limit:90+limit]) # Variable characterizing the distance between the robot and the obstacles on the left
        right = np.min(msg.ranges[270-limit:270+limit]) # Variable characterizing the distance between the robot and the obstacles on the right
        back = np.min(msg.ranges[180-limit:180+limit]) # Variable characterizing the distance between the robot and the obstacles behind

        front_l = np.min(msg.ranges[40-limit1:40+limit1]) # Variable characterizing the distance between the robot and the obstacles in front left
        front_r = np.min(msg.ranges[320-limit1:320+limit1]) # Variable characterizing the distance between the robot and the obstacles in front right
        
	# get speed parameters : linear and angular repectively
        linear_scale2 = rospy.get_param("linear_scale2")    
        angular_scale2 = rospy.get_param("angular_scale2")

        #move the robot forward and around
        if (front>dis1 and front_l>dis and front_r>dis) or (front>dis1 and front_l<dis and front_r<dis):
            # go forward
            #passing linear path
            vel_msg.linear.x = linear_scale2
            vel_msg.angular.z = 0
            vel_pub.publish(vel_msg)
        elif (front<dis1 and front_l>dis and front_r<dis) or (front>dis and front_l>dis and front_r<dis):
            # turn left
            # turning: not passing a linear path
            vel_msg.linear.x = 0
            vel_msg.angular.z = angular_scale2
            vel_pub.publish(vel_msg) 
        elif (front<dis1 and front_l>dis and front_r>dis) or (front>dis1 and front_l<dis and front_r>dis) or (front<dis1 and front_l<dis and front_r>dis):
            # turn right
            # turning: not passing a linear path
            vel_msg.linear.x = 0 #linear speed is zero
            vel_msg.angular.z = (-1)*angular_scale2 
            vel_pub.publish(vel_msg)        
        elif front<dis1 and front_l<dis and front_r<dis: # if there is obstacle at 3 direction
            if back>dis:  # if there is no obstacle behind
                # back
                vel_msg.linear.x = (-1)*linear_scale2
                vel_msg.angular.z = 0
                vel_pub.publish(vel_msg) 
            if front_l>front_r: 
                # turn left
                vel_msg.linear.x = 0
                vel_msg.angular.z = angular_scale2
                vel_pub.publish(vel_msg) 
            else:
                # turn right
                vel_msg.linear.x = 0
                vel_msg.angular.z = (-1)*angular_scale2
                vel_pub.publish(vel_msg)

        else:
            #print("ELSE [",front,front_l,front_r,"]")
            pass



# callback function for image Int16 (topic /connect)
def callback_red(msg):
    global couloir
    if msg.data == 6:    # if the robot arrives at the corridor, couloir = True
        couloir = True
       
    else:
        couloir=False

    


         
if __name__ == '__main__':
    
    try: 
        # Set a global variable n
        global couloir
       
        #Initialize the node called "challenge2"
        rospy.init_node('challenge2', anonymous=True)

        #create a publiser to publise the topic 'cmd_vel',the type of message is Twist and the length of queue is 10
        vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # create a subscriber receive the topic /connect, the type of msg is Int16 and then give to the callback_red function
        sub_red = rospy.Subscriber('/connect',Int16, callback_red)

        #create a subscriber receive the topic /scan, the type of msg is LaserScan and then give to the callback function
        rospy.Subscriber('/scan', LaserScan, callback)

        # keeps the node from exiting until the node has been shutdown
        rospy.spin()
            
          
    
    except rospy.ROSInterruptException:
        pass
