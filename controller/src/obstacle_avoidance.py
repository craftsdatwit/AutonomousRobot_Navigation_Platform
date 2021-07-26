#Logic Robot Control
#Purpose: To implement the robot control class to avoid objects and collisions
#Authors: David Crafts & Jake Sousa 


from Logic_Robot_Control_Class import RobotControl
import rospy
import time


class ObstacleAvoidance():

    #Initalize obstacle avoidance. Calls robotControl class
    def avoid_obstacle(self,name):
        
        rc = RobotControl(name) #Create new robotControl object with specific robot name passed 

        #Initialize threshold variables
        stopDistance = 1.0 #3.5ft
        scanDistance = 1.8
        movespeed = 0.12
        slowDownDistance = 1.524 #5ft
        currentspeed = 0.0


        #First scan quadrants
        rc.check_Top_Right_Clear(scanDistance)
        rc.check_Bottom_Right_Clear(scanDistance)
        rc.check_Top_Left_Clear(scanDistance)
        rc.check_Bottom_Left_Clear(scanDistance)

        #Initalize booleans
        waited = False
        turning = False
        slowdown = False
        turnedRight = False
        turnedLeft = False
      
        #While the code is getting a steady LiDAR feed
        while rc.check_Laser_Ready(name) == True:
            
            #If an object is detected in the emergency distance threshold. Stop robot
            if rc.check_Center_Clear(stopDistance) == False and slowdown ==  False and waited == False and turning == False:
                print("*EMERGENCY STOP* Object Detected at Distance: ")
                print(rc.return_Center_Blocked_Distance())
                rc.stop_Robot()
            
            #If an object is detected in the slowdown distance threshold. Slow down robot
            if rc.check_Center_Clear(slowDownDistance) == False and slowdown == False and turning == False:
                print("Object Detected within slow down distance --> Slowing Down")
                rc.slow_Down()
                time.sleep(0.2)
                waited = False
                slowdown = True
                        

            #If center is blocked and the robot hasn't waited  --> stop robot, wait 5 seconds, set waited to true
            if rc.check_Center_Clear(stopDistance) == False and waited == False and slowdown == True and turning == False:
                rc.stop_Robot()
                print("Object Detected within stopping distance --> Waiting for 5 seconds")
                time.sleep(2)
                waited = True

            #If top right is clear and the robot has waited and the robot is not turning --> turning is true, turn robot to the right and check center clear with stopDistance
            if rc.check_Top_Right_Clear(scanDistance) == True and waited == True and turning == False:
                print("Top Right is clear --> Navigating to top right")
                turning = True
                turnedRight = True
                rc.turn_Until_Clear("right", stopDistance)
           
            #If top left is clear and the robot has waited and the robot is not turning --> turning is true, turn robot to the left and check center clear with stopDistance
            if rc.check_Top_Left_Clear(scanDistance) == True and rc.check_Top_Right_Clear(scanDistance) == False and waited == True and turning == False:
                print("Top Left is clear --> Navigating to top left")
                turning = True
                turnedLeft = True
                rc.turn_Until_Clear("left", stopDistance)
            
                        
            #If bottom right is clear and the robot has waited and the robot is not turning --> turning is true, turn robot to the right and check center clear with stopDistance
            if rc.check_Bottom_Right_Clear(scanDistance) == True and rc.check_Top_Left_Clear(scanDistance) == False and rc.check_Top_Right_Clear(scanDistance) == False and waited == True and turning == False:
                print("Bottom Right is clear --> Navigating to bottom right")
                turning = True
                turnedRight = True
                rc.turn_Until_Clear("right", stopDistance)

            #If bottom left is clear and the robot has waited and the robot is not turning --> turning is true, turn robot to the left and check center clear with stopDistance
            if rc.check_Bottom_Left_Clear(scanDistance) == True and rc.check_Bottom_Right_Clear(scanDistance) == False and rc.check_Top_Left_Clear(scanDistance) == False and rc.check_Top_Right_Clear(scanDistance) == False and waited == True and turning == False:
                print("Bottom Left is clear --> Navigating to bottom left")
                turning = True
                turnedLeft = True
                rc.turn_Until_Clear("left", stopDistance)
            

            #if center is clear for given distance --> reset booleans, move robot straight
            elif rc.check_Center_Clear(stopDistance) == True:
                print("Center is clear --> Moving Forward")
                waited = False
                turning = False
                slowdown = False
                rc.move_Straight(movespeed)

                #While an object is detected on the right side and the robot turned left. Move straight to clear object
                while rc.check_Right_Side_Clear(0.8) == False and turnedLeft == True and rc.check_Center_Clear(0.8) == True:
                    print("Object Detected on Right Side Moving Straight")
                    rc.check_Laser_Ready(name)
                    print(turnedLeft)
                
                #While an object is detected on the left side and the robot turned left. Move straight to clear object
                while rc.check_Left_Side_Clear(0.8) == False and turnedRight == True and rc.check_Center_Clear(0.8) == True:
                    print("Object Detected on Left Side Moving Straight")
                    rc.check_Laser_Ready(name)
                    print(turnedRight)

                #Stop robot an reset turning booleans
                rc.stop_Robot()
                turnedLeft = False
                turnedRight = False
                return True
