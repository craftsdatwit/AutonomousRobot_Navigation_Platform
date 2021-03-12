from robot_control import RobotControl
from geometry_msgs.msg import Twist 
import rospy
import time

rc = RobotControl()

rc.move_straight()
time.sleep(5)
rc.stop_robot()

