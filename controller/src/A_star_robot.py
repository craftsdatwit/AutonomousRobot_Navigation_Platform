#!/usr/bin/env python3
#Code adapted from Ryan Collingwood (gist.github.com/ryancollingwood/32446307e976a11a1185a5394d6657bc)
#Modified by William McLellan


import rospy
import heapq
import numpy

from Logic_Robot_Control_Class import RobotControl
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

PI=3.14151926535897

x = 0.0
y = 0.0
theta = 0.0
stopDistance = 0.5 
scanDistance = 1.3
movespeed = 0.2
slowDownDistance = 1.524 #5ft
currentspeed = 0.0

rc = RobotControl()

rc.check_Top_Right_Clear(scanDistance)
rc.check_Bottom_Right_Clear(scanDistance)
rc.check_Top_Left_Clear(scanDistance)
rc.check_Bottom_Left_Clear(scanDistance)

#Initalize booleans
waited = False
turning = False
slowdown = False

def newOdom (msg):
	global x
	global y
	global theta
	
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	
	rot_q = msg.pose.pose.orientation
	(roll, pitch, theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

#rospy.init_node ("speed_controller")

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel",Twist, queue_size=1)

speed = Twist()

r = rospy.Rate(3)

goal = Point ()
#goal.x = 0
#goal.y = 0
    

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
    
    # Create lists for open nodes and closed nodes
    open = []
    closed = []
    # Create a start node and an goal node
    start_node = Node(start, None)
    goal_node = Node(end, None)
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
            # Return reversed path
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
            neighbor = Node(next, current_node)
            # Check if the neighbor is in the closed list
            if(neighbor in closed):
                continue
            # Generate heuristics (Manhattan distance)
            neighbor.g = abs(neighbor.position[0] - start_node.position[0]) + abs(neighbor.position[1] - start_node.position[1])
            neighbor.h = abs(neighbor.position[0] - goal_node.position[0]) + abs(neighbor.position[1] - goal_node.position[1])
            neighbor.f = neighbor.g + neighbor.h
            # Check if neighbor is in open list and if it has a lower f value
            if(add_to_open(open, neighbor) == True):
                # Everything is green, add neighbor to open list
                open.append(neighbor)
    # Return None, no path is found
    return None
# Check if a neighbor should be added to open list
def add_to_open(open, neighbor):
    for node in open:
        if (neighbor == node and neighbor.f >= node.f):
            return False
    return True
# The main entry point for this module
def main():
    rc.check_Laser_Ready()
    turning = False
    center_Clear = True
    # Get a map (grid)
    map = {}
    chars = ['c']
    start = None
    end = None
    width = 0
    height = 0
    # Open a file
    fp = open('/home/devuser/catkin_ws/src/Pallet_Project/controller/src/warehouse.txt', 'r')
    
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
    path = astar_search(map, start, end)
    print()
    print(path)
    print()
    draw_grid(map, width, height, spacing=1, path=path, start=start, goal=end)
    print()
    print('Steps to goal: {0}'.format(len(path)))
    print()
    print(path)
    #path.reverse()
    #print(path)
    
    ite = 0 #iteration
    while ite < len(path):
        tempx=path[ite][0]
        tempx = tempx - 10
        tempy=path[ite][1]
        tempy = tempy - 10
        
        path[ite]= (tempx, -tempy) #-y becuase it runs the opposite way then expected
        ite += 1
        
    print(path)
    
    point_index = 0
    while not rospy.is_shutdown():
        if point_index < len(path):
            goal.x = path[point_index][0]
            goal.y = path[point_index][1]
            
        else:
            break
            
        inc_x = goal.x - x
        inc_y = goal.y - y
        
        angle_to_goal = atan2 (inc_y, inc_x)
        distance_to_goal = numpy.sqrt(inc_x*inc_x + inc_y*inc_y)

        calc_angle = angle_to_goal - theta

        if distance_to_goal >= 0.5:
            if calc_angle > 5:
                calc_angle = calc_angle - (2*PI)
            if calc_angle < -5:
                calc_angle = calc_angle + (2*PI)
            if calc_angle > 0.2 and turning == False and center_Clear == True:
                speed.linear.x = 0.0
                turning = True
                speed.angular.z = 0.1
                #print(theta)
                print('I am turning left ' + str(calc_angle))
            if calc_angle < -0.2 and turning == False and center_Clear == True:
                speed.linear.x = 0.0
                turning = True
                speed.angular.z = -0.1
                print("I am turning right " + str(calc_angle))
                #print(theta)
            if calc_angle > -0.2 and calc_angle < 0.2 :
                if rc.check_Center_Clear(stopDistance) == False:
                    #rc.stop_Robot()
                    speed.linear.x = 0.0
                    speed.angular.z = 0.0
                    center_Clear = False
                    print('Object Detected')
                if rc.check_Center_Clear(stopDistance) ==True:
                    print("Center is clear")
                    center_Clear = True
                    turning = False
                    speed.linear.x = 0.15
                    speed.angular.z = 0.0
                    print("I am moving straight " + str(calc_angle))
                
            pub.publish(speed)
        else:
            rc.slow_Down()
            point_index += 1
        r.sleep()

    rc.stop_Robot()
    
# Tell python to run main method
if __name__ == "__main__": main()


