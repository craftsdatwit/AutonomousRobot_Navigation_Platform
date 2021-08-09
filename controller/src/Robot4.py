#Purpose: Creates an instance of the pathFollowing algorithm with a unique robot name and map

from Robot_Path_Following import pathFollowing

name2 = "robot4"

yoffset = 0

warehouseName = "./Warehouse Map Files/robot4demo.txt"

pf2 = pathFollowing(name2, warehouseName, yoffset) #Passes in robot name and warehouse map path file to the pathFollowing oject

pf2.main(name2,warehouseName, yoffset) #Calls the main method with those same variables