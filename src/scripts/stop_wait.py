#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def stopandwait(msg):
    distances = msg.ranges

    #Define stopping distance
    stopDistance = 1.2

    #Define center ranges
    center = distances[300:400]
    minDistance = min(center)

    #Set initial speed
    move.linear.x = 0.2

    #Check if min distance is less than 1.2 meters away
    if minDistance < stopDistance:
       #This will stop the robot
        move.linear.x = 0.0

    pub.publish(move)
    

    

#Initialize node 
rospy.init_node('turtlebot3_waffle_pi_stop_and_wait')

#Creating message type to be passed into publisher
move = Twist()

#Initialize publisher, which outputs commands to turtlebot
pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

#Initlaize subscriber, which reades in the LIDAR data from the turtlebot LIDAR
sub = rospy.Subscriber('/scan',LaserScan, stopandwait)

rospy.spin()