from Logic_Robot_Control_Class import RobotControl
import rospy
import time

stopDistance = 0.7 #3.5ft
scanDistance = 1.3
movespeed = 0.15
slowDownDistance = 1.524 #5ft
currentspeed = 0.0



rc = RobotControl()

while rc.check_Laser_Ready() == True:


    if rc.check_Left_Side_Clear(stopDistance) == False:
        print("Object Detected on Left Side")

    if rc.check_Left_Side_Clear(stopDistance) == True:
        print("Object Not Detected on Left Side")


    if rc.check_Right_Side_Clear(stopDistance) == False:
        print("Object Detected on Right Side")

    if rc.check_Right_Side_Clear(stopDistance) == True:
        print("Object Not Detected on Right Side")
rospy.spin()