#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def goaround(msg):
    
    def turnRight():
        #Turn Right
        move.linear.x = 0.0
        move.angular.z = -0.1
        pub.publish(move)
        time.sleep(8)
    
    def turnLeft():
        #Turn left
        move.linear.x = 0.0
        move.angular.z = 0.1
        pub.publish(move)
        time.sleep(8)
    
    def goStraight(timeToWait):
        move.angular.z = 0.0
        move.linear.x = 0.2
        pub.publish(move)
        time.sleep(timeToWait)
        
    
    def stopMoving():
        move.angular.z = 0.0
        move.linear.x = 0.0
        pub.publish(move)

    def getDistances():
        return msg.ranges
    
    def minCenterDistance():
        distances = getDistances()
        left = distances[348:359]
        right = distances[0:11]
        return min(left+right)
    

    global waited

    #Define stopping distance
    stopDistance = 1.2

    #Get closest distance from center ranges
    minDistance =  minCenterDistance()

    if minDistance >= stopDistance:
        waited = False
        print("I will go forward")
        goStraight(0)

    if minDistance < stopDistance:
        
        if waited:
            print("I have waited and will start moving around object")
            #Stop robot
            stopMoving()

            #Move right
            turnRight()

            #Go straight
            goStraight(7)

            #Turn Left
            turnLeft()

            #Go straight
            goStraight(7)

            #Turn left
            turnLeft()

            #Go Straight
            goStraight(7)

            #Turn Right
            turnRight()

            #Reset flag
            waited = False
            time.sleep(3)
            
        elif not waited:
            print("I will start waiting")
            #Stop robot
            stopMoving()

            #Set flag
            waited = True

            #Set time
            time.sleep(5)

    
waited = None

#Initialize node 
rospy.init_node('turtlebot3_waffle_pi_stop_and_go_around')

#Creating message type to be passed into publisher
move = Twist()

#Initialize publisher, which outputs commands to turtlebot
pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

#Initlaize subscriber, which reades in the LIDAR data from the turtlebot LIDAR
sub = rospy.Subscriber('/scan',LaserScan, goaround)

rospy.spin()