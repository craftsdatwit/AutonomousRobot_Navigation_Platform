#Logic Robot Control
#Purpose: To implement the robot control class to avoid objects and collisions
#Authors: David Crafts & Jake Sousa 


from Logic_Robot_Control_Class import RobotControl
import rospy
import time

#stopDistance = 0.7
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

#Initalize booleans
waited = False
turning = False
slowdown = False

#While the laser scanner is recieving data
while rc._check_laser_ready() == True:
    
    if rc.check_Center_Clear(slowDownDistance) == False and slowdown == False:
        print("Slowing Down")
        rc.slow_down()
        waited = False
        slowdown = True
        #turning = False
                

    #If center is blocked and the robot hasn't waited  --> stop robot, wait 5 seconds, set waited to true
    if rc.check_Center_Clear(stopDistance) == False and waited == False and slowdown == True:
        rc.stop_robot()
        print("Object Detected --> Waiting for 5 seconds")
        time.sleep(5)
        waited = True

    #If top right is clear and the robot has waited and the robot is not turning --> turning is true, turn robot to the right and check center clear with stopDistance
    if rc.check_Top_Right_Clear(stopDistance) == True and waited == True and turning == False:
        print("Top Right is clear --> Navigating to top right")
        turning = True
        rc.turn_Until_Clear("right", scanDistance)

    if rc.check_Top_Left_Clear(stopDistance) == True and rc.check_Top_Right_Clear(stopDistance) == False and waited == True and turning == False:
        print("Top Left is clear --> Navigating to top left")
        turning = True
        rc.turn_Until_Clear("left", scanDistance)
    
    if rc.check_Bottom_Right_Clear(stopDistance) == True and rc.check_Top_Left_Clear(stopDistance) == False and rc.check_Top_Right_Clear(stopDistance) == False and waited == True and turning == False:
        print("Bottom Right is clear --> Navigating to bottom right")
        turning = True
        rc.turn_Until_Clear("right", stopDistance)

    if rc.check_Bottom_Left_Clear(stopDistance) == True and rc.check_Bottom_Right_Clear(stopDistance) == False and rc.check_Top_Left_Clear(stopDistance) == False and rc.check_Top_Right_Clear(stopDistance) == False and waited == True and turning == False:
        print("Bottom Left is clear --> Navigating to bottom left")
        turning = True
        rc.turn_Until_Clear("left", stopDistance)

    #if center is clear for given distance --> reset booleans, move robot straight
    elif rc.check_Center_Clear(slowDownDistance) == True and rc.check_Center_Clear(stopDistance) == True:
        print("Center is clear --> Moving Forward")
        waited = False
        turning = False
        slowdown = False
        rc.move_straight()

