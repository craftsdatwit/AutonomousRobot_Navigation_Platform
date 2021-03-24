from robot_control import RobotControl
from geometry_msgs.msg import Twist 
import rospy
import time

rc = RobotControl()

counter = 5

while counter >= 0:
    print("Counter:", counter)
    rc.turn("coutner", 1, 3)

    rc.move_straight()
    time.sleep(12)
    rc.stop_robot()

    counter= counter -1