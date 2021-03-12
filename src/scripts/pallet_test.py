from robot_control import RobotControl
from geometry_msgs.msg import Twist 
import rospy
import time

rc = RobotControl() #Declaration of a robot control object

#list1 = rc.get_laser_full()
#left_list = rc.sensor_left()
timeout = time.time() + 20

center_list = rc.sensor_center() #gets the min sensor value from range

while center_list > 1: #While robot is more than 1m away from object
    center_list = rc.sensor_center() #Get sensor value
    rc.move_straight() #move robot straight

    while center_list < 1: #While robot is less than 1m away from object
        center_list = rc.sensor_center() #Get sensor value
        #left_list = rc.sensor_left()
        rc.stop_robot() #stop robot
        print("Distance to wall: ", center_list) #print distance to wall


rc.stop_robot()

"""

for x in center_list:
    if x > 1:
        center_list = rc.sensor_center()
        rc.move_straight()
        print("Distance: ", x)
    if x < 1:
        rc.stop_robot()



rc.stop_robot()
"""

"""
while list1[360] > 1: #while robot is more than 1m from object
    list1 = rc.get_laser_full() #get laser feed
    rc.move_straight() #move robot straight

    if list1[360] < 1: #if robot is less than 1m from object
        list1 = rc.get_laser_full() #get laser feed
        rc.stop_robot() #stop robot
        print("Distance to wall: ", list1[360]) #print distance to wall

        if list1[719] > 1:
            rc.turn("counterclockwise", 0.5, 2)
            rc.move_straight()


rc.stop_robot()

"""