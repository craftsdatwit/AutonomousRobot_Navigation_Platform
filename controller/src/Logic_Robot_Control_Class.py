#!/usr/bin/env python
#Code adapted from TheConstructSim (theconstructsim.com)
#Modified by David Crafts and Jake Sousa


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
import decimal

class RobotControl():

    def __init__(self, robot_name="turtlebot3"):

        #Initalize robot control node 
        rospy.init_node('robot_control_node', anonymous=True) 

        #Print to screen 
        rospy.loginfo("Robot Turtlebot3...") 

        #Topic variable for cmd velocity
        cmd_vel_topic='/cmd_vel' 

        # We start the publisher for cmd velocity
        self.vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)

        #Initalize a new object of type Twist
        self.cmd = Twist() #Initalize a new object of type Twist

        #Create a subscriber that subscribes to the /scan topic
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.laser_Callback) 
      

        #Check that LiDAR Sensor is ready
        self.check_Laser_Ready()

        self.ctrl_c = False

        self.rate = rospy.Rate(10)

        rospy.on_shutdown(self.shutdownhook)
    
    #Checks that sensor is ready
    def check_Laser_Ready(self):
        self.laser_msg = None
        rospy.loginfo("Checking Laser...")
        while self.laser_msg is None and not rospy.is_shutdown():
            try:
                self.laser_msg = rospy.wait_for_message("/scan", LaserScan, timeout=1.0)
                rospy.logdebug("Current /scan READY=>" + str(self.laser_msg))
                rospy.loginfo("Checking Laser...READY")
                time.sleep(0)

            except:
                rospy.logerr("Current /scan not ready yet, retrying for getting scan")
        rospy.loginfo("Checking Laser...COMPLETE")
        return True

    
    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems, this is no big deal, but in systems that publish only
        once, it IS very important.
        """
        while not self.ctrl_c:
            connections = self.vel_publisher.get_num_connections()
            if connections > 0:
                self.vel_publisher.publish(self.cmd)
                #rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

    # /// LASER FUNCTIONS \\\

    def laser_Callback(self, msg):
        self.laser_msg = msg


    #Gets laser at a given position
    def get_Laser(self, pos):
        time.sleep(1)
        return self.laser_msg.ranges[pos]

    #Returns entire laser array
    def get_Laser_Full(self):
        time.sleep(0)
        return self.laser_msg.ranges

    #Checks if center is clear for a given distance
    def check_Center_Clear(self, distance): 
        ranges = self.laser_msg.ranges
        right = ranges[346:359]
        left = ranges[0:13]

        center = left+right

        # Returns False if an object is detected
        if min(center) < distance:
            return False
        #Returns true if no object is detected
        if min(center) > distance:
            return True
       
    #Returns the distance from robot to an object if the center is blocked
    def return_Center_Blocked_Distance(self):
        ranges = self.laser_msg.ranges
        right = ranges[346:359]
        left = ranges[0:13]

        center = left+right

        return min(center)


    #Check if top right is clear
    def check_Top_Right_Clear(self, scanDistance):
        topright = self.laser_msg.ranges[270:344] #Get the right side range

        maxr = max(topright) #Gets the max value of the right side range
        indexr = topright.index(maxr) #Gets the index of the maxr value
        success =0 #Success variable
        toprightfree = False

        #If maxr is greater than the scan distance
        if maxr > scanDistance:
            success +=1 
            for i in topright[indexr:]: #for i in the right index starting at the max index 
                if i > scanDistance: #if i is greater than the scan distance
                    success +=1 
                else: #if in the scan there's an object 
                    break
                if success == 28: #if an opening has been found that's wide enough for the robot
                    break
            if success == 28: #if an opening is found
                #print("Top right opening found")
                success = 0
                toprightfree = True
            else: #if no opening is found
                #print("No top right opening found")
                success = 0 #reset success variable
                toprightfree = False
        return toprightfree

        #Check if bottom right is clear
    def check_Bottom_Right_Clear(self, scanDistance):
        bottomright = self.laser_msg.ranges[181:269]

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
                if success == 28: 
                    break
            if success == 28: 
                #print("Bottom right opening found")
                success = 0
                bottomrightfree = True
            else: 
                #print("No bottom right opening found")
                success = 0
                bottomrightfree = False
            return bottomrightfree

    #Check if top left is clear
    def check_Top_Left_Clear(self, scanDistance):
        left = self.laser_msg.ranges[15:90] 

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
                if success == 28:
                    break
            if success == 28:
                #print("Top left opening found")
                success = 0
                leftFree = True
            else:
                #print("No top left opening found")
                success = 0
                leftFree = False
        return leftFree

    #Check if bottom left is clear
    def check_Bottom_Left_Clear(self, scanDistance):
        bottomleft = self.laser_msg.ranges[91:179] 

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
                if success == 28:
                    break
            if success == 28:
                #print("Bottom left opening found")
                success = 0
                bottomleftFree = True
            else:
                #print("No bottom left opening found")
                success = 0
                bottomleftFree = False
        return bottomleftFree


     # /// MOVEMENT FUNCTIONS \\\

    #Stops the robot
    def stop_Robot(self):
        #rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    #Slows down the robot's movement rapidly
    def slow_Down(self):
        for i in self.float_Range(0.2,0.05,0.00001):
            self.cmd.linear.x = i
            if i < 0.05:
                self.cmd.linear.x = 0.05
            currentspeed = i
            self.vel_publisher.publish(self.cmd)
            #time.sleep(0.00000001)
            
    #Helper function for slow_Down
    def float_Range(self,start,stop,step):
        while start > stop:
            yield float(start)
            start -= step

    #Moves the robot straight
    def move_Straight(self, moveSpeed):

        # Initilize velocities
        self.cmd.linear.x = moveSpeed
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        # Publish the velocity
        self.publish_once_in_cmd_vel()
        
    #Moves robot straight for specific parameters *Not used, depreciated function*
    def move_Straight_Time(self, motion, speed, time):

        # Initilize velocities
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        if motion == "forward":
            self.cmd.linear.x = speed
        elif motion == "backward":
            self.cmd.linear.x = - speed

        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (i <= time):

            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            i += 1
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Moved robot " + motion + " for " + str(time) + " seconds"
        return s

    #Turns the robot in a specific direction until an opening is found
    def turn_Until_Clear(self, direction, stopDistance):

        if direction == "left" and self.check_Center_Clear(stopDistance) == False:
            print("Turning left")
            self.cmd.angular.z = 0.2
            self.vel_publisher.publish(self.cmd)
        

        if direction == "right" and self.check_Center_Clear(stopDistance) == False:
            print("Turning right")
            self.cmd.angular.z = -0.2
            self.vel_publisher.publish(self.cmd)
        
    #Turns the robot either right or left with a given speed
    def turn_Direction(self, direction, turnSpeed):
        
         if direction == "left":
            print("Turning left")
            self.cmd.linear.x = 0
            self.cmd.angular.z = turnSpeed
            self.vel_publisher.publish(self.cmd)
        

         if direction == "right":
            print("Turning right")
            self.cmd.linear.x = 0
            self.cmd.angular.z = -turnSpeed
            self.vel_publisher.publish(self.cmd)

    #Turn robot based on specific parameters *Not used, depreciated function*
    def turn(self, clockwise, speed, time):

        # Initilize velocities
        self.cmd.linear.x = 0
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0

        if clockwise == "clockwise":
            self.cmd.angular.z = -speed
        else:
            self.cmd.angular.z = speed

        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (i <= time):

            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            i += 1
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Turned robot " + clockwise + " for " + str(time) + " seconds"
        return s


if __name__ == '__main__':
    
    robotcontrol_object = RobotControl()
    try:
        robotcontrol_object.move_straight()

    except rospy.ROSInterruptException:
        pass
