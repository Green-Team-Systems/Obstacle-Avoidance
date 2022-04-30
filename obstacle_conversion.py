# ========================================================================================
# Created By: Dulani Wijayarathne
# Authors: Dulani Wijayarathne
# Created On: March 11th, 2022
# Last Modified: April 25th, 2022
# 
# Description: Module to convert the obstacles in a numpy map to run in RRT algorithm 
# ========================================================================================

import pandas
import numpy as np


# Load the voxel grid map
map = np.load('3darr.npy')

rows = 100
cols = 100
# Converting the obstacles of the map from boolean logic to 3D coordinates

obstacles = []

# loop to iterate through the map
for x in range(0,100):
    for y in range(0,100):
        for z in range(0,100):
            if (map[x][y][z]):
                obstacles.append([x,y,z])

# Initializing the variables for the conversion algorithm
count = 0 # Keeping track of the coordinate points that belong to the same obstacle
temp = 0 # Temporary variable to save the count of the coordinates that belong to the same obstacle
obs = [] # obstacle list
templist = [] # Temporary variable for the obstacle list
count_list = []

# Outer most for loop that iterate over all the obstacles
for i in range(len(obstacles)):
    if (i != (len(obstacles) - 1)): # condition to iterate until one obstacle before the last obstacle
        if (obstacles[i+1][2] == ((obstacles[i][2]) + 1)): # condition to check the sequence of each obstacle using the z coordinate
            count = count + 1 # count of the obstacle in the same z sequence
            templist.append(obstacles[i]) # appending the a subset of obstacles with the same consequence 
        else:
                if ((obstacles[i-1][2]) + 1) == (obstacles[i][2]): # condition to check if the coordinates belongs to the same subsest of obstacle
                    count = count + 1 # increasing the count of the sequence
                    if temp == 0:
                        templist.append(obstacles[i])
                        i = i+1
                        temp = count # storing the count of the sequence to a temporaray variable
                        count = 0 
                        
                    else:
                        if ((obstacles[i - count][2] == obstacles[i][2])): # condtion to check if the obstacles are of the same consequence
                                templist.append(obstacles[i]) # appending the a subset of obstacles with the same consequence 
                                i = i+1
                                if (temp != count): # condition to check if the obstacle list belongs to the same subset of obstacles
                                        preobs = templist[-count:] # making a list of the obstacles of different sequence
                                        del templist[-count:] # deleting the obstacles of different sequence
                                        obs.append(templist) # appending the subset of obstacles
                                        count_list.append(count) # appending the count of the sequence 
                                        templist = [] # making a temporary list of the coordinate sequence of the obstacles
                                        templist = preobs # appending the obstacles 
                                temp = count # storing the count of the sequence to a temporaray variable
                                count = 0 # setting the count of the obstalce to zero


    else:       
                if ((obstacles[i-1][2]) + 1) == (obstacles[i][2]): # condition to check if the last obstacle in the list belong to a subset of obstacle
                    count = count + 1 # increasing the count of the sequence
                    templist.append(obstacles[i]) # appeding to a temporary list
                    
                    if (temp != count): # condition to check if count belongs to a subset
                        preobs = templist[-count:]
                        del templist[-count:]
                        obs.append(templist)
                        count_list.append(count)
                    else:
                        obs.append(templist)
                        count_list.append(count) # appending the obstacle to a list
        

obstaclesrrt = [] # making cuboides using the maximum and the minimum corner of the obstacle

# saving the coordinates of the obstacles to a list
pd = pandas.DataFrame(obs)
pd.to_csv("obstacles.csv")

# loop to iterate through the obstacles
count = 0
for i in obs:
        # listing all the coordinates of the obstacle
        xs = [x[0] for x in i]
        ys = [y[1] for y in i] 
        zs = [z[2] for z in i] 

        # deleting the last sequence with a 0 in the y plane
        if (ys[len(ys) - 1] == 0):
            del ys[-(count_list[count]):]
     
        # condition to check the maximum and the minium corner of the obstacle
        if (max(xs) > min(xs)) & (max(ys) > min(ys)) & (max(zs) > min(zs)):
            obstaclesrrt.append((min(xs),min(ys),min(zs),max(xs),max(ys),max(zs))) # appending the obstacle to send to rrt algorithm if the maximum and the minimum corner are not the same        
         
        else:

            max_xs = max(xs) 
            max_ys = max(ys)
            max_zs = max(zs)

            # if the minimum and the maximum corner of the obstacle are in 2d, increasing one unit in one plane to convert to 3d
            if (max(xs) == min(xs)):
                max_xs = max(xs) + 1
            if (max(ys) == min(ys)):
                max_ys = min(ys) + 1
            if (max(zs) == min(zs)):
                max_zs = min(zs) + 1
            obstaclesrrt.append((min(xs),min(ys),min(zs), max_xs,max_ys,max_zs))
        count = count + 1