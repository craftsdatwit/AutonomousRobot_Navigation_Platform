#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def goaround(msg):

    stopDistance = 1.0

    scanDistance = 2.0
    

    def turnRight():
        #Turn Right
        move.linear.x = 0.0
        move.angular.z = -0.3
        pub.publish(move)

    def turnLeft():
        #Turn left
        move.linear.x = 0.0
        move.angular.z = 0.3
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
        right = distances[348:359]
        left = distances[0:11]
        return min(right+left)

    #Check if top right is clear
    def checkTopRight():
        topright = msg.ranges[270:347] #Get the right side range

        maxr = max(topright) #Gets the max value of the right side range
        indexr = topright.index(maxr) #Gets the index of the maxr value
        success =0 #Success variable
        toprightfree = False

        #If maxr is greater than the stop distance
        if maxr > scanDistance:
            success +=1 
            for i in topright[indexr:]: #for i in the right index starting at the max index 
                if i > scanDistance: #if i is greater than the stop distance
                    success +=1 #add one to I
                else: #if in the scan there's an object 
                    break
                if success == 21: #if an opening has been found
                    break
            if success == 21: #if an opening is found
                #print("Top right opening found")
                success = 0
                toprightfree = True
            else: #if no opening is found
                #print("No top right opening found")
                success = 0
                toprightfree = False
        return toprightfree

    #Check if bottom right is clear
    def checkBottomRight():
        bottomright = msg.ranges[181:269]

        maxr = max(bottomright) 
        indexr = bottomright.index(maxr) 
        success =0 
        bottomrightfree = False

        if maxr > scanDistance:
            success +=1 
            for i in bottomright[indexr:]: 
                if i > scanDistance:
                    success +=1 
                else: 
                    break
                if success == 21: 
                    break
            if success == 21: 
                #print("Bottom right opening found")
                success = 0
                bottomrightfree = True
            else: 
                #print("No bottom right opening found")
                success = 0
                bottomrightfree = False
            return bottomrightfree

    #Check if top left is clear
    def checkTopLeft():
        left = msg.ranges[12:90] 

        maxl = max(left)
        indexl = left.index(maxl)
        success = 0
        leftFree = False

        if maxl > scanDistance:
            success +=1
            for i in left[indexl:]:
                if i > scanDistance:
                    success +=1
                else:
                    break
                if success == 21:
                    break
            if success == 21:
                #print("Top left opening found")
                success = 0
                leftFree = True
            else:
                #print("No top left opening found")
                success = 0
                leftFree = False
        return leftFree

    #Check if bottom left is clear
    def checkBottomLeft():
        bottomleft = msg.ranges[91:179] 

        maxl = max(bottomleft)
        indexl = bottomleft.index(maxl)
        success = 0
        bottomleftFree = False

        if maxl > scanDistance:
            success +=1
            for i in bottomleft[indexl:]:
                if i > scanDistance:
                    success +=1
                else:
                    break
                if success == 21:
                    break
            if success == 21:
                #print("Bottom left opening found")
                success = 0
                bottomleftFree = True
            else:
                #print("No bottom left opening found")
                success = 0
                bottomleftFree = False
        return bottomleftFree


    global waited
    global turning
    global topleft 
    global topright
    global bottomright 
    global bottomleft 



    #Get closest distance from center ranges
    center =  minCenterDistance()


    if center < stopDistance and not waited:
        #print("Center is blocked and has not waited")
        #Stop robot
        #stopMoving()

        #SCAN
        topright = checkTopRight()
        topleft = checkTopLeft()
        bottomright = checkBottomRight()
        bottomleft = checkBottomLeft()

        #Set flag
       # print("Waiting now true")
        waited = True

        #SCAN
        topright = checkTopRight()
        topleft = checkTopLeft()
        bottomright = checkBottomRight()
        bottomleft = checkBottomLeft()

        #Set time
        time.sleep(0)



        #If the center is blocked
    if center < stopDistance and waited:
        #print("If center is blocked and has waited")
        topright = checkTopRight()
        topleft = checkTopLeft()
        bottomright = checkBottomRight()
        bottomleft = checkBottomLeft()

        if topright == True and not turning: #if right is free
            print("Going top right")
            turning = True 
            turnRight()
         
            #time.sleep(2)
            #goStraight()

        elif topright == False and topleft == True and not turning:
            turning = True
            turnLeft()
            print("Going top left")

        elif topright == False and topleft == False and bottomright == True and not turning:
            turning = True
            turnRight()
            print("Going bottom right")

        elif topright == False and topleft == False and bottomright == False and bottomleft == True and not turning:
            turning = True
            
            turnLeft()
            print("Going bottom left")
        

    elif center >= stopDistance:
        waited = False
        turning = False #reset turning
       # print("Center is not blocked, moving forward")
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