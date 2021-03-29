#https://github.com/enansakib/obstacle-avoidance-turtlebot/blob/master/src/naive_obs_avoid_tb3.py
#!/usr/bin/env python

import rospy # Python library for ROS
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist #Twist message for robot movement
import time

def callback(dt):
  
    laser_threshold = 1.3 # Laser scan range threshold in meters

    def getMinCenter():
        m_dist_cent2 = dt.ranges[339:359] #ranges center and slightly left
        m_dist_cent1 =  dt.ranges[0:20] #ranges cener and slightly right

        #adds left and right lists together and returns minimum value at any point 
        #getting the minimum value means any of the center values are blocked. 
        return min(m_dist_cent2 + m_dist_cent1)



    #This gets the right sensor values 
    def getLeft():
       return dt.ranges[21:110]

    #This gets the left sensor values
    def getRight():
       return dt.ranges[250:338] 

    #This gets the min left sensor values 
    def getMinLeft():
       return min(dt.ranges[21:110])

    #This gets the left sensor values
    def getMinRight():
       return min(dt.ranges[250:338]) 

    #This gets the maximum sensor value for any given left point
    def getMaxRight():
       return max(dt.ranges[250:338])

    #This gets the maximum sensor value for any given right point
    def getMaxLeft():
       return max(dt.ranges[21:110])


    m_dist_cent = getMinCenter() #Get minimum center list 
    dist_right = getRight() #Get right list 
    dist_left = getLeft() #Get left list 

    max_dist_right = getMaxRight()
    max_dist_left = getMaxLeft()

    min_dist_left = getMinLeft()
    min_dist_right = getMinRight()

    if m_dist_cent > laser_threshold: 

        move.angular.z = 0.0
        move.linear.x = 0.1 
        print("Moving")
        
    #if center is blocked
    if m_dist_cent < laser_threshold: 

        move.linear.x = 0.0 #stop robot
        move.angular.z = 0.0
        print("Center is blocked: Stopping")
        
        if max_dist_right > laser_threshold:
            print(max_dist_right)
            print("Opening found right")
            move.angular.z = -0.2 #rotate clockwise
            #move.linear.x = 0.0 #don't move forward

        elif max_dist_left > min_dist_right:
            print("Opening found left")
            move.angular.z = 0.2
            #move.linear.x = 0.0

    pub.publish(move) # publish the move object



move = Twist() # Creates a Twist message type object
rospy.init_node('obstacle_avoidance_node') # Initializes a node
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  # Publisher object which will publish "Twist" type messages
                            				 # on the "/cmd_vel" Topic, "queue_size" is the size of the
                                                         # outgoing message queue used for asynchronous publishing

sub = rospy.Subscriber("/scan", LaserScan, callback)  # Subscriber object which will listen "LaserScan" type messages
                                                      # from the "/scan" Topic and call the "callback" function
						      # each time it reads something from the Topic

rospy.spin() # Loops infinitely until someone stops the program execution

