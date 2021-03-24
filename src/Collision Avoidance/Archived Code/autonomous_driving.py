#Autonomous Robot control
#Written by David Crafts

from robot_control import RobotControl
from geometry_msgs.msg import Twist 
import rospy
import time

#Robot Control declaration
rc = RobotControl() 

def callback(rc):
    if rc.get_laser[0] > 1 and rc.get_laser[15] > 1 and rc.get_laser[345] > 1:

        rc.move_straight()
    else:
        rc.stop_robot()
        rc.turn("cc", 0.3, 1)

        if rc.get_laser[0] > 1 and rc.get_laser[15] > 1 and rc.get_laser[345] > 1:
            rc.move_straight()

