#Purpose: Creates an instance of the pathFollowing algorithm with a unique robot name and map

from Logic_Robot_Path_Following import pathFollowing

name1 = "robot1"

yoffset = 0

warehouseName = "warehouse_test.txt"

pf1 = pathFollowing(name1, warehouseName, yoffset) #Passes in robot name and warehouse map path file to the pathFollowing oject

pf1.main(name1,warehouseName, yoffset) #Calls the main method with those same variables