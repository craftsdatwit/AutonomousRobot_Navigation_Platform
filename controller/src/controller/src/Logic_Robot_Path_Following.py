#!/usr/bin/env python3
#Code adapted from Ryan Collingwood (gist.github.com/ryancollingwood/32446307e976a11a1185a5394d6657bc)
#Modified by William McLellan, David Crafts and Jake Sousa


import rospy
import heapq
import numpy
import time

from Logic_Robot_Control_Class import RobotControl
from obstacle_avoidance import ObstacleAvoidance
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, pi


class pathFollowing():
    
    #Initialize pathFollowing Class
    def __init__(self,name,warehouseFile):

        #Variables for point navigation
        x = 0.0
        y = 0.0
        theta = 0.0

        #variables for robot control
        stopDistance = 0.7 
        scanDistance = 1.3
        movespeed = 0.12
        slowDownDistance = 1.524 #5ft
        currentspeed = 0.0
        speed = Twist()
        distance_to_goal = 0.0
        calc_angle = 0.0
        checkDistance = 1.0
        turnDistance = 0.3


    #set odometry data for robot position
    def newOdom (self,msg):
        global x
        global y
        global theta

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])   


    # This class represents a node
    class Node():
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
    def draw_grid(self,map, width, height, spacing=2, **kwargs):
        for y in range(height):
            for x in range(width):
                print('%%-%ds' % spacing % self.draw_tile(map, (x, y), kwargs), end='')
            print()

    # Draw a tile
    def draw_tile(self,map, position, kwargs):
        
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
            
    # A* search algorithm for path finiding
    def astar_search(self,map, start, end):

        # Create lists for open nodes and closed nodes
        open = []
        closed = []

        # Create a start node and an goal node
        start_node = self.Node(start, None)
        goal_node = self.Node(end, None)

        # Add the start node
        open.append(start_node)
        
        # Loop until the open list is empty
        while len(open) > 0:
            # Sort the open list to get the node with the lowest cost first
            open.sort()
            # Get the node with the lowest cost
            current_node = open.pop(0)
            # Add the current node to the closed list
            closed.append(current_node)
            
            # Check if we have reached the goal, return the path
            if current_node == goal_node:
                path = []
                while current_node != start_node:
                    path.append(current_node.position)
                    current_node = current_node.parent
                #path.append(start) 
                #Return reversed path
                return path[::-1]
            # Unzip the current node position
            (x, y) = current_node.position
            # Get neighbors
            neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]
            # Loop neighbors
            for next in neighbors:
                # Get value from map
                map_value = map.get(next)
                # Check if the node is a wall
                if(map_value == '1'):
                    continue
                # Create a neighbor node
                neighbor = self.Node(next, current_node)
                # Check if the neighbor is in the closed list
                if(neighbor in closed):
                    continue
                # Generate heuristics (Manhattan distance)
                neighbor.g = abs(neighbor.position[0] - start_node.position[0]) + abs(neighbor.position[1] - start_node.position[1])
                neighbor.h = abs(neighbor.position[0] - goal_node.position[0]) + abs(neighbor.position[1] - goal_node.position[1])
                neighbor.f = neighbor.g + neighbor.h
                # Check if neighbor is in open list and if it has a lower f value
                if(self.add_to_open(open, neighbor) == True):
                    # Everything is green, add neighbor to open list
                    open.append(neighbor)
        # Return None, no path is found
        return None
            
    # Check if a neighbor should be added to open list
    def add_to_open(self,open, neighbor):
        for node in open:
            if (neighbor == node and neighbor.f >= node.f):
                return False
        return True


    #Calculates angle measurements
    def angleMeasurements(self,path, goal):
        
        goal.x = path[0][0]
        goal.y = path[0][1]
        #calculate distance and angle to goal    
        inc_x = goal.x - x
        inc_y = goal.y - y
            
        angle_to_goal = atan2 (inc_y, inc_x)
        distance_to_goal = numpy.sqrt(inc_x*inc_x + inc_y*inc_y)

        #desired angle - robots acual angle
        calc_angle = angle_to_goal - theta
        return (calc_angle,distance_to_goal)
            
    # The main entry point for this class
    def main(self, name, warehouseFile):

        #Call objects
        pf = pathFollowing(name,warehouseFile) #New pathFollowing object
        rc = RobotControl(name) #New robotControl object
        roa = ObstacleAvoidance() #New obstacleAvoidance object

        goal = Point() #Goal is of data type point

        #Create odom name based off of unique robot name. Initialize subscriber to odom topic
        odomName = "/%s/odom" % (name)
        sub = rospy.Subscriber(odomName, Odometry, pf.newOdom) 

        #Check individual robot LiDAR status
        rc.check_Laser_Ready(name)

        #Initialize booleans / variables. Distance in M
        turning = False
        center_Clear = True
        obstacle = False
        global distance_to_goal
        global calc_angle
        stopDistance = 0.7 
        scanDistance = 1.3
        movespeed = 0.12
        slowDownDistance = 1.524 #5ft


        # Get a map (grid)
        map = {}
        chars = ['c']
        start = None
        end = None
        width = 0
        height = 0
            
        # Open the warehouse grid with start and navigate too points
        #fp = open('/home/devuser/catkin_ws/src/Pallet_Project/controller/src/warehouse.txt', 'r')
        #secondary file for chnaging points without taking away base setup
        fp = open(warehouseFile, 'r')
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
                    start = (z, height)
                elif(chars[z] == '$'):
                    end = (z, height)
                
            # Increase the height of the map
            if(len(chars) > 0):
                height += 1
            # Close the file pointer
        fp.close()
            # Find the closest path from start(@) to end($)
        path = pf.astar_search(map, start, end)
            
        print()
        pf.draw_grid(map, width, height, spacing=1, path=path, start=start, goal=end)
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
        
        #Iterate through each point, and move to that point
        point_index = 0
        while not rospy.is_shutdown():
            if point_index < len(path):
                measurements = pf.angleMeasurements(path, goal)
                calc_angle = measurements[0]
                distance_to_goal = measurements[1]
            else:
                break
            
            

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
                    rc.turn_Direction("left", 0.08)
                    print('Robot Status: Turning left to align with next node')

                #Right turn
                if calc_angle < -0.2 and turning == False and center_Clear == True:
                    turning = True
                    rc.turn_Direction("right", 0.08)
                    print("Robot Status: Turning right to align with next node" )

                #Move forward if robot is facing goal node
                if calc_angle > -0.2 and calc_angle < 0.2 :
                    #If center is blocked by an obstacle for slowdown distance threshold
                    if rc.check_Center_Clear(slowDownDistance) == False:

                        rc.slow_Down() #Slow robot

                        #If object is still blocking center for stop distance threshold
                        if rc.check_Center_Clear(stopDistance) == False:

                            print("Robot Status: Center Not Clear")
                            print("Calling Collision Avoidance Function")

                            roa.avoid_obstacle(name) #Call collision avoidance

                            #Pop's the next 2 nodes off of the path stack
                            path.pop(0)
                            path.pop(0)
                            
                            #Recalculate angles to next node
                            measurements = pf.angleMeasurements(path, goal) 
                            calc_angle = measurements[0]
                            distance_to_goal = measurements[1]


                            print("CALC ANGLE: "+ str(calc_angle) + "DISTANCE: "+ str(distance_to_goal))
                            print("AFTER OBSTACLE PATH: " + str(path))
                            print("NEW COORDINATES: x: " + str(x) + " y: " + str(y))
                            #rc.move_Straight(0.15)
                            #continue
                    
                    #If center is clear, move robot straight
                    if rc.check_Center_Clear(stopDistance) ==True:
                        print("Robot Status: Center is Clear")
                        center_Clear = True
                        turning = False
                        rc.move_Straight(0.12)
                        #print("I am moving straight " + str(calc_angle))
                
                    #print("Obstacle: " + str(obstacle))

            #If at the next point
            else:
                rc.slow_Down() #Slow down robot
                #point_index += 1
                path.pop(0) #If at next point pop that point of node stack
                print("NEW PATH: " + str(path)) #Print new path
        #When at final point, stop moving
        rc.stop_Robot()


    
# Tell python to run main method
if __name__ == "__main__": main()