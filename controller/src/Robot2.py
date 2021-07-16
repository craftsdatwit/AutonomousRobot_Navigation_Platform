#Purpose: Creates an instance of the pathFollowing algorithm with a unique robot name and map

from Logic_Robot_Path_Following import pathFollowing

name2 = "robot2"

warehouseName = "warehouse2.txt"

pf2 = pathFollowing(name2, warehouseName) #Passes in robot name and warehouse map path file to the pathFollowing oject

pf2.main(name2,warehouseName) #Calls the main method with those same variables