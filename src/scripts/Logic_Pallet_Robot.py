#Logic Robot Control
#Purpose: To implement the robot control class to avoid objects and collisions
#Authors: David Crafts & Jake Sousa 


from Logic_Robot_Control_Class import RobotControl
import rospy
import time

stopDistance = 1.067 #3.5ft
scanDistance = 2.0
movespeed = 0.2
slowDownDistance = 1.2 #5ft
currentspeed = 0.0



rc = RobotControl()



rc.check_Top_Right_Clear(scanDistance)
rc.check_Bottom_Right_Clear(scanDistance)
rc.check_Top_Left_Clear(scanDistance)
rc.check_Bottom_Left_Clear(scanDistance)



while rc._check_laser_ready() == True:
    
    #If center is blocked
    while rc.check_Center_Clear(stopDistance) == False:
        rc.stop_robot()

        if rc.check_Top_Right_Clear(stopDistance) == True:
            rc.turn_Until_Clear("right", stopDistance)
    
    if rc.check_Center_Clear(stopDistance) == True:
        print("Center is clear")
        rc.move_straight()

