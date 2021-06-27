#!/usr/bin/env python3
#Code adapted from Ryan Collingwood (gist.github.com/ryancollingwood/32446307e976a11a1185a5394d6657bc)
#Modified by William McLellan, David Crafts and Jake Sousa


import rospy
import heapq
import numpy
import time

from Logic_Robot_Control_Class import RobotControl
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, pi

#Variables for point navigation
x = 0.0
y = 0.0
floatX = 0.0
floatY = 0.0
theta = 0.0
goal = Point ()
#variables for robot control
stopDistance = 0.5 
scanDistance = 1.3
clearDistance = 1.6
movespeed = 0.2
slowDownDistance = 1.524 #5ft
currentspeed = 0.0
speed = Twist()



#set odometry data for robot position
def newOdom (msg):
    global x
    global y
    global floatX
    global floatY
    global theta

    x = msg.pose.pose.position.x
    #print("FROM SUBSCRIBER X:" + str(x))
    y = msg.pose.pose.position.y
    #print("FROM SUBSCRIBER Y: " + str(y))
    floatX = float(x)
    floatY = float(y)
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])   

sub = rospy.Subscriber("/odom", Odometry, newOdom) 

# This class represents a node
class Node:
    # Initialize the class
    def __init__(self, position:(), parent:()):
        self.position = position
        self.parent = parent
        self.g = 0 # Distance to start node
        self.h = 0 # Distance to goal node
        self.f = 0 # Total cost
    # Compare nodes
    def __eq__(self, other):
        return self.position == other.position
    # Sort nodes
    def __lt__(self, other):
         return self.f < other.f
    # Print node
    def __repr__(self):
        return ('({0},{1})'.format(self.position, self.f))
        
# Draw a grid
def draw_grid(map, width, height, spacing=2, **kwargs):
    for y in range(height):
        for x in range(width):
            print('%%-%ds' % spacing % draw_tile(map, (x, y), kwargs), end='')
        print()
# Draw a tile
def draw_tile(map, position, kwargs):
    
    # Get the map value
    value = map.get(position)
    # Check if we should print the path
    if 'path' in kwargs and position in kwargs['path']: value = '+'
    # Check if we should print start point
    if 'start' in kwargs and position == kwargs['start']: value = '@'
    # Check if we should print the goal point
    if 'goal' in kwargs and position == kwargs['goal']: value = '$'
    # Return a tile value
    return value 
    
# A* search
def astar_search(map, start, end):
    print(" A STAR CALLED")
    # Create lists for open nodes and closed nodes
    open = []
    closed = []
    # Create a start node and an goal node
    #print("BEFORE START NODE")
    start_node = Node(start, None)
    #print("AFTER START NODE")
    #print("BEFORE END NODE")
    goal_node = Node(end, None)
    #print("AFTER END NODE")
    # Add the start node
    open.append(start_node)
    #print("AFTER APPEND START NODE TO OPEN")
    
    # Loop until the open list is empty
    while len(open) > 0:
        #print("INSIDE WHILE LEN > 0")
       #print(len(open))
        # Sort the open list to get the node with the lowest cost first
        open.sort()
        # Get the node with the lowest cost
        current_node = open.pop(0)
        # Add the current node to the closed list
        closed.append(current_node)
        
        # Check if we have reached the goal, return the path
        if current_node == goal_node:
            print("Reached goal node")
            path = []
            while current_node != start_node:
                path.append(current_node.position)
                current_node = current_node.parent
            #path.append(start) 
            # Return reversed path
            return path[::-1]
        # Unzip the current node position
        (x, y) = current_node.position
        # Get neighbors
        neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]
        # Loop neighbors
        for next in neighbors:
            #print("INSIDE FOR LOOP")
            # Get value from map
            map_value = map.get(next)
            #print("AFTER GETTING MAP VALUE")
            # Check if the node is a wall
            if(map_value == '1'):
                #print("IF NEIGHBOR IS A WALL")
                continue
            # Create a neighbor node
            neighbor = Node(next, current_node)
            # Check if the neighbor is in the closed list
            if(neighbor in closed):
                continue
            # Generate heuristics (Manhattan distance)
            #print("BEFORE CALC")
            neighbor.g = abs(neighbor.position[0] - start_node.position[0]) + abs(neighbor.position[1] - start_node.position[1])
            neighbor.h = abs(neighbor.position[0] - goal_node.position[0]) + abs(neighbor.position[1] - goal_node.position[1])
            #print("AFTER CALC")
            neighbor.f = neighbor.g + neighbor.h
            # Check if neighbor is in open list and if it has a lower f value
            if(add_to_open(open, neighbor) == True):
                #print("ADDING NEIGHBOR TO OPEN LIST")
                # Everything is green, add neighbor to open list
                open.append(neighbor)
    # Return None, no path is found
    #print("NO PATH WAS FOUND")
    return None
    
# Check if a neighbor should be added to open list
def add_to_open(open, neighbor):
    for node in open:
        if (neighbor == node and neighbor.f >= node.f):
            return False
    return True
    
# The main entry point for this module
def main():
    global path
    global end
    
    #Call in RobotControl Class
    rc = RobotControl()
    #set up lasers and booleons for robot control
    rc.check_Laser_Ready()
    turning = False
    center_Clear = True
    
    # Get a map (grid)
    map = {}
    chars = ['c']
    start = None
    end = None
    iStart = None
    iEnd = None
    width = 0
    height = 0
    
    # Open the warehouse grid with start and navigate too points
    #fp = open('/home/devuser/catkin_ws/src/Pallet_Project/controller/src/warehouse.txt', 'r')
    #secondary file for chnaging points without taking away base setup
    fp = open('/media/sf_Pallet_Project/controller/src/warehouse.txt', 'r')
    # Loop until there is no more lines
    while len(chars) > 0:
        # Get chars in a line
        chars = [str(i) for i in fp.readline().strip()]
        # Calculate the width
        width = len(chars) if width == 0 else width
        # Add chars to map
        for z in range(len(chars)):
            map[(z, height)] = chars[z]
            if(chars[z] == '@'):
                start = (float(z), float(height))
                iStart = (z,height)
            elif(chars[z] == '$'):
                end = (float(z), float(height))
                iEnd = (z,height)
        
        # Increase the height of the map
        if(len(chars) > 0):
            height += 1
    # Close the file pointer
    fp.close()
    # Find the closest path from start(@) to end($)
    path = astar_search(map, start, end)
    
    print()
    draw_grid(map, width, height, spacing=1, path=path, start=start, goal=end)
    print()
    print('Steps to goal: {0}'.format(len(path)))
    print()
        
    
    #The graph starts at (0,0) in the middle of a 21 by 21 grid
    #This section iterates through the points to adjust for not starting at a corner
    ite = 0 #iteration
    while ite < len(path):
        tempx=path[ite][0]
        tempx = tempx - 10 #starting point is (0,0), but thinks its at (11,11) 
        tempy=path[ite][1]
        tempy = tempy - 10
        #-y becuase it runs the opposite way then expected
        path[ite]= (tempx, -tempy) 
        ite += 1
    #To see all points along the path in list form   
    print(path)
    #print("Original END GOAL: " + end)
    
    #Iterate through each point, and move to that point
    point_index = 0
    while not rospy.is_shutdown():
        if point_index < len(path):
            goal.x = path[point_index][0]
            goal.y = path[point_index][1]
            
        else:
            break
        
        #calculate distance and angle to goal    
        inc_x = goal.x - x
        inc_y = goal.y - y
        
        angle_to_goal = atan2 (inc_y, inc_x)
        distance_to_goal = numpy.sqrt(inc_x*inc_x + inc_y*inc_y)

        #desired angle - robots acual angle
        calc_angle = angle_to_goal - theta
        #Initalize booleans
        waited = False
        turning = False
        slowdown = False
        objectInPath = False
        turnedLeft = False
        turnedRight = False
        #if the robot is not a certian distance away from goal point
        if distance_to_goal >= 0.3:
            #Robot had issues at the extremes, +-2*Pi, this offsets number
            #lowest was 5.9, so 5 is generous and should never have issues 
            if calc_angle > 5:
                calc_angle = calc_angle - (2*pi)
            if calc_angle < -5:
                calc_angle = calc_angle + (2*pi)
            #Left turn
            if calc_angle > 0.2 and turning == False and center_Clear == True:
                turning = True
                rc.turn_Direction("left", 0.1)
                #print('I am turning left ' + str(calc_angle))
                
            #Right turn
            if calc_angle < -0.2 and turning == False and center_Clear == True:
                turning = True
                rc.turn_Direction("right", 0.1)
                #print("I am turning right " + str(calc_angle))
               #print("Original Start" + str(x) + str(y))
            #Move forward
            if calc_angle > -0.2 and calc_angle < 0.2 :
                
                if rc.check_Center_Clear(slowDownDistance) == False:
                    objectInPath = True

                    while objectInPath == True:

                        if rc.check_Center_Clear(stopDistance) == False and slowdown ==  False and waited == False and turning == False:
                            #print("*EMERGENCY STOP* Object Detected at Distance: ")
                            #print(rc.return_Center_Blocked_Distance())
                            rc.stop_Robot()
                        
                        if rc.check_Center_Clear(slowDownDistance) == False and slowdown == False and turning == False:
                            #print("Object Detected within slow down distance --> Slowing Down")
                            rc.slow_Down()
                            time.sleep(0.2)
                            waited = False
                            slowdown = True
                            #turning = False

                        #If center is blocked and the robot hasn't waited  --> stop robot, wait 5 seconds, set waited to true
                        if rc.check_Center_Clear(stopDistance) == False and waited == False and slowdown == True and turning == False:
                            rc.stop_Robot()
                            #print("Object Detected within stopping distance --> Waiting for 5 seconds")
                            time.sleep(0)
                            waited = True

                        #If top right is clear and the robot has waited and the robot is not turning --> turning is true, turn robot to the right and check center clear with stopDistance
                        if rc.check_Top_Right_Clear(scanDistance) == True and waited == True and turning == False:
                            #print("Top Right is clear --> Navigating to top right")
                            turnedRight = True
                            turning = True
                            rc.turn_Until_Clear("right", stopDistance)

                        if rc.check_Top_Left_Clear(scanDistance) == True and rc.check_Top_Right_Clear(scanDistance) == False and waited == True and turning == False:
                            #print("Top Left is clear --> Navigating to top left")
                            turnedLeft = True
                            turning = True
                            rc.turn_Until_Clear("left", stopDistance)
                        
                        if rc.check_Bottom_Right_Clear(scanDistance) == True and rc.check_Top_Left_Clear(scanDistance) == False and rc.check_Top_Right_Clear(scanDistance) == False and waited == True and turning == False:
                            #print("Bottom Right is clear --> Navigating to bottom right")
                            turning = True
                            rc.turn_Until_Clear("right", stopDistance)

                        if rc.check_Bottom_Left_Clear(scanDistance) == True and rc.check_Bottom_Right_Clear(scanDistance) == False and rc.check_Top_Left_Clear(scanDistance) == False and rc.check_Top_Right_Clear(scanDistance) == False and waited == True and turning == False:
                            #print("Bottom Left is clear --> Navigating to bottom left")
                            turning = True
                            rc.turn_Until_Clear("left", stopDistance)
                        
                        elif rc.check_Center_Clear(slowDownDistance) == True and rc.check_Center_Clear(stopDistance) == True:
                            #print("Center is clear --> Moving Forward")
                           
                            waited = False
                            turning = False
                            slowdown = False
                            rc.move_Straight(movespeed)
                            if turnedRight == True:
                                #print("I TURNED RIGHT")
                                if rc.check_Top_Left_Clear(stopDistance) == True:
                                    while rc.check_Top_Left_Clear(clearDistance) == False:
                                        rc.move_Straight(movespeed)

                                    while rc.check_Bottom_Left_Clear(clearDistance) == False:
                                        rc.move_Straight(movespeed)
                                

                                    print("***** CHECK CENTER Left CLEAR *****")
                                    rc.stop_Robot()
                                    newStart = None
                                    #print("Left OLD START: " + str(x) + str(y))
                                    print("Left I MADE IT") 
                                    newStart = (floatX,floatY)
                                    print("Left NEW START: " + str(x) + str(y))
                            
                                    objectInPath = False
                                    """
                                        Below is where I go through and delete all of the old nodes in the list
                                        There still needs to be a check that will make sure that our current local is not behind the first node in the list/
                                    
                                    """
                                    print("OLD PATH: " + str(path))
                                    for i in range(0,point_index+1):
                                        path.pop(i)
                                    #path = astar_search(map, newStart, end)
                                    print("Left I MADE IT 2")
                                    print("NEW PATH: " + str(path))
                                    
                                    point_index = 0 # Reset index to start back at top of the list
                            if turnedLeft == True:
                                #print("I TURNED LEFT")

                                while rc.check_Top_Right_Clear(scanDistance) == False:
                                    rc.move_Straight(movespeed)

                                while rc.check_Bottom_Right_Clear(scanDistance) == False:
                                    rc.move_Straight(movespeed)
                                

                                print("***** CHECK CENTER RIGHT CLEAR *****")
                                rc.stop_Robot()
                                newStart = None
                                #print("OLD START: " + str(x) + str(y))
                                print("I MADE IT") 
                                print("OLD PATH: " + str(path))
                                newStart = (floatX,floatY)
                                print("NEW START: " + str(x) + str(y))
                        
                                objectInPath = False

                                """
                                        Below is where I go through and delete all of the old nodes in the list
                                        There still needs to be a check that will make sure that our current local is not behind the first node in the list/
                                    
                                """
                                for i in range(0,point_index+1):
                                        path.pop(i)
                                print("I MADE IT 2")
                                print("NEW PATH: " + str(path))
 
                                point_index = 0 # Reset index to start back at top of the list

                                
                            else:
                                rc.move_Straight(movespeed)
                    
                
                if rc.check_Center_Clear(stopDistance) ==True:
                    #print("Center is clear")
                    center_Clear = True
                    turning = False
                    turnedLeft = False
                    turnedRight = False
                    rc.move_Straight(0.15)
                    #print("I am moving straight " + str(calc_angle))
                
        #If at the next point
        else:
            rc.slow_Down()
            point_index += 1
    #When at final point, stop moving
    rc.stop_Robot()
    
# Tell python to run main method
if __name__ == "__main__": main()


