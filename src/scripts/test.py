from robot_control import RobotControl
from geometry_msgs.msg import Twist 
import rospy
import time

rc = RobotControl()

rc.move_straight()
center_list = rc.sensor_center() #gets the min sensor value from range

print("Center List", center_list)