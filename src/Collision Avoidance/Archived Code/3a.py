#https://github.com/enansakib/obstacle-avoidance-turtlebot/blob/master/src/naive_obs_avoid_tb3.py
#!/usr/bin/env python
import rospy # Python library for ROS
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist #
from collections import defaultdict

def callback(dt):
  
    laser_threshold = 1.2 # Laser scan range threshold in meters

    def getMinCenter():
        m_dist_cent2 = dt.ranges[345:359] #ranges center and slightly left
        m_dist_cent1 =  dt.ranges[0:15] #ranges cener and slightly right

        #adds left and right lists together and returns minimum value at any point 
        #getting the minimum value means any of the center values are blocked. 
        return min(m_dist_cent2 + m_dist_cent1)
    
    def getRight():
       return dt.ranges[16:110] #This gets the right sensor values 

    def getLeft():
       return dt.ranges[250:344] #This gets the left sensor values

    def getMaxLeft():
       return max(dt.ranges[250:349]) #This gets the maximum sensor value for any given left point

    def getMaxRight():
       return max(dt.ranges[16:110]) #This gets the maximum sensor value for any given right point


    m_dist_cent = getMinCenter() #Get minimum center list 
    dist_right = getRight() #Get right list 
    dist_left = getLeft() #Get left list 


    if m_dist_cent < laser_threshold: #if center is blocked
        move.linear.x = 0.0 #stop robot

        #CHECK RIGHT SIDE

        for i in m_dist_right: #Check each angle on the right side

            if i > laser_threshold: #if angle on the right side is greater than the threshold
                move.angular.z = -0.3 #rotate clockwise
                move.linear.x = 0.0 #don't move forward
<<<<<<< HEAD:src/Collision Avoidance/Archived Code/3a.py
            else:
                for k in m_dist_left: #for each left value
                    if k > laser_threshold: #if left value is greater than threshold
=======
		
            else: #else (if right is less than)
                for k in m_dist_left: #for each angle in left
                    if k > thr1: #if angle is greater than threshold (no object detected
>>>>>>> ee853efd62dc396c1f838e601f5f69a5eaebef90:src/scripts/3a.py
                        move.angular.z = 0.3 #rotate counter clockwise
                        move.linear.x =0

    else:
        move.linear.x = 0.3 #move forward 0.3m/s
        move.angular.z = 0.0 #don't turn

            



    """
    #dt.ranges[0]>thr1 and dt.ranges[15]>thr2 and dt.ranges[345]>thr2:
    if m_dist_cent>thr1: # Checks if there are obstacles in front and
                                                                   # 15 degrees left and right (Try changing the
									 # the angle values as well as the thresholds)

        #move.linear.x = 0.3 # go forward (linear velocity)
       
        move.angular.z = 0.0 # do not rotate (angular velocity)

        
    else:
        move.linear.x = 0.0 # stop robot
        print("Stopping")
        m_dist_cent =  getMinCenter()
        
        if m_dist_cent<thr1: #if center distance to object is less than threshold 
            move.linear.x = 0.0 #stop robot 
            print("Stopping if")
        

        if m_dist_right > 1.2: #if right distance to object is greater than threshold
            move.linear.x =0.0 #stop robot
            print("Right side distance more than threshold")
            move.angular.z = -0.5 #move clockwise
            move.linear.x =0.2 #move forward

        elif m_dist_right< 1.5:
                move.linear.x =0.0 #stop robot
                print("Right less than threshold")
                if m_dist_left > 1.5:
                    print("Left is greater than threshold")
                    move.angular.z = 0.5 #move clockwise
                    move.linear.x =0.2 #move forward
        
        else:
            move.linear.x = 0.3

        
        if dt.ranges[0] > thr1:
            move.linear.x = 0.4
        """

    pub.publish(move) # publish the move object



move = Twist() # Creates a Twist message type object
rospy.init_node('obstacle_avoidance_node') # Initializes a node
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # Publisher object which will publish "Twist" type messages
                            				 # on the "/cmd_vel" Topic, "queue_size" is the size of the
                                                         # outgoing message queue used for asynchronous publishing

sub = rospy.Subscriber("/scan", LaserScan, callback)  # Subscriber object which will listen "LaserScan" type messages
                                                      # from the "/scan" Topic and call the "callback" function
						      # each time it reads something from the Topic

rospy.spin() # Loops infinitely until someone stops the program execution

