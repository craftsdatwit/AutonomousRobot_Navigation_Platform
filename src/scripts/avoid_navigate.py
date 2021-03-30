#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def goaround(msg):

    stopDistance = 1.2
    
    rightFree = None
    
    def turnRight():
        #Turn Right
        move.linear.x = 0.0
        move.angular.z = -0.1
        pub.publish(move)

    
    def turnLeft():
        #Turn left
        move.linear.x = 0.0
        move.angular.z = 0.1
        pub.publish(move)

    
    def goStraight():
        move.angular.z = 0.0
        move.linear.x = 0.2
        pub.publish(move)
        #time.sleep(timeToWait)
        
    
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

    #Check if the right side is free only if the section is wide enough for the robot
    def checkRight():
        print("Checking Right")

        #Get the right side range
        right = msg.ranges[270:347] 

        #Gets the max value of the right side range
        maxr = max(right)

        #Gets the index of the maxr value
        indexr = right.index(maxr)

        #Success variable
        success =0

        #If maxr is greater than the stop distance
        if maxr > stopDistance:
            print(maxr) 
            success +=1 #add 1 to success variable
            for i in right[indexr:]: #for i in the right index starting at the max index 
                if i > stopDistance: #if i is greater than the stop distance
                    #print(i)
                    success +=1 #add one to I
                else:
                    break
                if success == 20:
                    break
            if success == 20: #if an opening is found
                print("Success, opening found: ", success)
                success = 0
                rightFree = True
            else: #if no opening is found
                #print("Failure no path found")
                success = 0
                rightFree = False
        print(success)
        return rightFree
    
    def checkLeft():
        print("Checking Left")
        left = msg.ranges[12:90] 

        maxl = max(left)
        indexl = left.index(maxl)
        success = 0

        if maxl > stopDistance:
            print(maxl)
            success +=1
            for i in left[indexl:]:
                if i > stopDistance:
                    #print(i)
                    success +=1
                else:
                    break
                if success == 20:
                    break
            if success == 20:
                print("Success path found: ", success)
                success = 0
                leftFree = True
            else:
                #print("Failure no path found")
                success = 0
                leftFree = False
        print(success)
        return leftFree


    global waited

    #Define stopping distance
    stopDistance = 1.2

    #Get closest distance from center ranges
    center =  minCenterDistance()

        
    if center < stopDistance and waited:
        print("If center is blocked and has waited")

        left = checkLeft()
        right = checkRight()

        if right == True:
            print("Turning Right")
            turnRight()
            #time.sleep(2)
            #goStraight()
        if right == False and left == True:
            turnLeft()
            print("Right blocked, turning left")

        
            
    if center < stopDistance and not waited:
        print("Center is blocked and has not waited")
        #Stop robot
        stopMoving()

        #Set flag
        print("Waiting now true")
        waited = True

        #Set time
        time.sleep(3)

    elif center >= stopDistance:
        waited = False
        print("Center is not blocked, moving forward")
        goStraight()

    
waited = 0

#Initialize node 
rospy.init_node('turtlebot3_waffle_pi_stop_and_go_around')

#Creating message type to be passed into publisher
move = Twist()

#Initialize publisher, which outputs commands to turtlebot
pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

#Initlaize subscriber, which reades in the LIDAR data from the turtlebot LIDAR
sub = rospy.Subscriber('/scan',LaserScan, goaround)

rospy.spin()