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

#turning = False



rc.check_Top_Right_Clear(scanDistance)
rc.check_Bottom_Right_Clear(scanDistance)
rc.check_Top_Left_Clear(scanDistance)
rc.check_Bottom_Left_Clear(scanDistance)

waited = False

#While the laser scanner is recieving data
while rc._check_laser_ready() == True:
    
    #If center is blocked and the robot hasn't waited  --> stop robot, wait 5 seconds, set waited to true
    if rc.check_Center_Clear(stopDistance) == False and waited == False:
        rc.stop_robot()
        time.sleep(5)
        waited = True

    if rc.check_Top_Right_Clear(stopDistance) == True and waited == True:
        rc.turn_Until_Clear("right", stopDistance)

    if rc.check_Top_Left_Clear(stopDistance) == True and rc.check_Top_Right_Clear(stopDistance) == False and waited == True:
        rc.turn_Until_Clear("left", stopDistance)
    
    if rc.check_Bottom_Right_Clear(stopDistance) == True and rc.check_Top_Left_Clear(stopDistance) == False and rc.check_Top_Right_Clear(stopDistance) == False and waited == True:
        rc.turn_Until_Clear("right", stopDistance)
        print(rc.check_Center_Clear(stopDistance))

    if rc.check_Center_Clear(stopDistance) == True:
        print("Center is clear")
        waited = False
        rc.move_straight()

