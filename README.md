# RRT

RRT Implementation 

This module is designed to find the most optimal (short) path to a destination  with or without a series of obstacles to avoid. It provides 3D coordinate points that the drone should follow given the follwowing parameters. 

1. Numpy 3D Boolean array sent by mapping
2. Starting point and Ending Point

Relevant Files 

3darr.npy - A 3D numpy of a voxel grid map

obstacle_conversion - Load the obstacles from a numpy to the RRT algorithm

rrt_3d.py - Given an initial point and a goal location find the optimal path and print outs the coordinates of the trajectory. This is an existing 3D algorithm with modified parameters. 

Testing 

Import any 3D numpy file to the obstacle_conversion and change the dimensions manually. Run the rrt_3d.py file by initializing any intial location and goal location without obstacles on line 14 and 15.
