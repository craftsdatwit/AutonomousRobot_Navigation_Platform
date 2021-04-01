#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def goaround(msg):

    stopDistance = 2.0

    scanDistance = 3.5
    
    """
    bottomrightfree = None
    toprightfree = None
    leftFree = None
    """

    
    def turnRight():
        #Turn Right
        move.linear.x = 0.0
        move.angular.z = -0.2
        pub.publish(move)

    
    def turnLeft():
        #Turn left
        move.linear.x = 0.0
        move.angular.z = 0.2
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
    def checkTopRight():
        #print("Checking Top Right")

        topright = msg.ranges[270:347] #Get the right side range

        maxr = max(topright) #Gets the max value of the right side range

        indexr = topright.index(maxr) #Gets the index of the maxr value

        success =0 #Success variable

        toprightfree = False

        #If maxr is greater than the stop distance
        if maxr > scanDistance:
            success +=1 #add 1 to success variable

            for i in topright[indexr:]: #for i in the right index starting at the max index 

                if i > stopDistance: #if i is greater than the stop distance
                    #print(i)
                    success +=1 #add one to I
                else: #if in the scan there's an object 
                    break

                if success == 20: #if an opening has been found
                    break
            if success == 20: #if an opening is found
                print("Success, opening found: ", success)
                success = 0
                toprightfree = True
            else: #if no opening is found
                #print("Failure no path found")
                success = 0
                toprightfree = False
        return toprightfree

    #Check if the right side is free only if the section is wide enough for the robot
    def checkBottomRight():
        #print("Checking Bottom Right")

        #Get the right side range
        bottomright = msg.ranges[181:269]

        #Gets the max value of the right side range
        maxr = max(bottomright) 

        #Gets the index of the maxr value
        indexr = bottomright.index(maxr) #Gets the index of the maxr value
        
        #Success variable
        success =0 

        bottomrightfree = False

        #If maxr is greater than the stop distance
        if maxr > scanDistance:
            print(maxr) 
            success +=1 #add 1 to success variable

            for i in bottomright[indexr:]: #for i in the right index starting at the max index 

                if i > stopDistance: #if i is greater than the stop distance
                    #print(i)
                    success +=1 #add one to I
                else: #if in the scan there's an object 
                    break

                if success == 20: #if an opening has been found
                    break
            if success == 20: #if an opening is found
                success = 0
                bottomrightfree = True
            else: #if no opening is found
                success = 0
                bottomrightfree = False
            return bottomrightfree


    def checkTopLeft():
        #print("Checking top Left")
    
        left = msg.ranges[12:90] 
        maxl = max(left)
        indexl = left.index(maxl)
        success = 0

        leftFree = False

        if maxl > scanDistance:
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
                success = 0
                leftFree = True
            else:
                success = 0
                leftFree = False
        return leftFree


    def checkBottomLeft():
        #print("Checking bottom Left")

        bottomleft = msg.ranges[91:179] 
        maxl = max(bottomleft)
        indexl = bottomleft.index(maxl)
        success = 0

        bottomleftFree = False

        if maxl > scanDistance:
            print(maxl)
            success +=1
            for i in bottomleft[indexl:]:
                if i > stopDistance:
                    #print(i)
                    success +=1
                else:
                    break
                if success == 20:
                    break
            if success == 20:
                success = 0
                bottomleftFree = True
            else:
                success = 0
                bottomleftFree = False
        return bottomleftFree


    global waited


    #Define stopping distance
    stopDistance = 1.2

    #Get closest distance from center ranges
    center =  minCenterDistance()

    #If the center is blocked
    if center < stopDistance and waited:
        #print("If center is blocked and has waited")

        topleft = checkTopLeft()
        topright = checkTopRight()
        bottomright = checkBottomRight()
        bottomleft = checkBottomLeft()

        if topright == True: #if right is free
            print("Turning Right")
            turnRight()
            #time.sleep(2)
            #goStraight()

        elif topright == False and topleft == True:
            turnLeft()
            print("Right blocked, turning left")

        elif topright == False and topleft == False and bottomright == True:
            turnRight()
            print("Top right and top left are blocked")

        elif topright == False and topleft == False and bottomright == False and bottomleft == True:
            turnLeft()
            print("Top Right / Left and Bottom Right are blocked")
        

        
            
    if center < stopDistance and not waited:
        print("Center is blocked and has not waited")
        #Stop robot
        stopMoving()

        #Set flag
        print("Waiting now true")
        waited = True

        #Set time
        time.sleep(1)

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