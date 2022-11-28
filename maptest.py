import setup_path 
import airsim
from enum import Enum

import sys
import math
import time
import argparse
import pprint
import numpy as np
import time
from enum import Enum
import json
import random


import matplotlib.pyplot as plt
import numpy as np

from itertools import groupby


# OA algorithm that generates new way points based on safety sphere around obstacles

class ClearPathObstacleAvoidance:

    destination = (-100, 50, 0)
    collisiondist = 4
    vehicle_name,lidar_names = 'Drone1',['LidarSensor1']

    # could use enum
    STATES = {
    "Clear": "TOGOAL",
    "Obstacle": "AVOID"
    }

    @property
    def estimated_kinematics(self):
        return self.client.getMultirotorState().kinematics_estimated


    def __init__(self):

        # connect to the AirSim simulator

        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.mode = self.STATES["Clear"]

    def takeoff(self):
        # Takeoff of Drone
        state = self.client.getMultirotorState()
        s = pprint.pformat(state)
        print(s)
        airsim.wait_key('Press any key to takeoff')
        self.client.takeoffAsync().join()
        state = self.client.getMultirotorState()

    def stop(self):

        airsim.wait_key('Press any key to reset to original state')

        self.client.armDisarm(False)
        self.client.reset()

        self.client.enableApiControl(False)
        print("Done!\n")

    """
    Description: This function is used to get the lidar point list data from the drone.
    
    Inputs:vehicle name,lidar names
    
    Outputs: 2d Array of 3d Lidar Data
    
    Notes: Array is grouped by rows, each row is a list of 3d points
    structured as follows - [[[x,y,z],[x,y,z],[x,y,z]], [[x,y,z],[x,y,z],[x,y,z]], [[x,y,z],[x,y,z],[x,y,z]]]
    """

    def scan(self):
        # print('Scanning ...')
        # Scan lidar data
        next_row = []
        y_points_last = 0
        overall_point_list=[]
        for lidar_name in self.lidar_names:

            lidar_data = self.client.getLidarData(lidar_name=lidar_name,vehicle_name=self.vehicle_name)
            data = lidar_data.point_cloud
            for i in range(0, len(lidar_data.point_cloud), 3):
                xyz = lidar_data.point_cloud[i:i+3]

                # Check at the end of each row for positive y and negative y value at the beginning of new row
                if (xyz[1] != math.fabs(xyz[1]) and y_points_last == math.fabs(y_points_last)):
                    overall_point_list.append(next_row)
                    next_row = list()
                next_row.append(xyz)
                y_points_last = xyz[1]
        return overall_point_list

    """
    Description: Find closest row parallel to the z level of the drone
    
    Inputs: overall_point_list
    
    Outputs: row index i
    
    Notes:
    """

    def chooseRow(self, overall_point_list):
        # print(overall_point_list)
        top_row = overall_point_list[1] 
        mid_point_top_level_ind = int(len(top_row) / 2)
        z_level_row = top_row[mid_point_top_level_ind][2] # initial z val of top row mid
        i = 1

        # row - [[x,y,z], [x,y,z], [x,y,z]]

        for row_index, row in enumerate(overall_point_list):
            if(len(row) > 0):
                mid_point = int(len(row) / 2) # index of mid point
                mid_point_z = row[mid_point][2] # z value of mid point
                if(abs(mid_point_z) < abs(z_level_row)):
                    z_level_row = mid_point_z
                    # print(z_level_row)
                    # print('Found new row')
                    i = row_index

        # print ("chose row" + str(i))
        return i

    """
    Description: detects gaps between points and returns row of lidar points with a gap symbol
    
    Inputs: threshold - distance between points, row of points
    
    Outputs: Fixed row of points
    
    Notes:
    """

    def gapLabel(self, threshold, chosen_row):

        #Todo: place 'Gap' at the end / corner


        last_point = np.array([chosen_row[0][0], chosen_row[0][1]])  #list of x and y value of first point
        for ind, point_val in enumerate(chosen_row):
            # [point_x, point_y] = np.array([point_val[0], point_val[1]])
            temp = last_point - np.array([point_val[0], point_val[1]])
            vector_dist = np.sqrt(temp[0]**2 + temp[1]**2)

            #calculated the vector distance between two points to see if there's a gap in the lidar data
            if vector_dist > threshold:
                # print('inserted')
                chosen_row.insert(ind, 'G')

            last_point = np.array([point_val[0], point_val[1]])

        #if there is no gap assume the end of lidar is the edge of the obstacle
        # chosen_row.append('E')

        return chosen_row


    def view_distance_filter(self, view_distance, fixed_row):

        #Todo: make function consider 'G'
        filtered_row = []
        for ind, val in enumerate(fixed_row):

            if val != 'G':
                dist = val[0]
                if(dist <= view_distance):
                    filtered_row.append(val)
        # x,y values of filtered row


        return filtered_row

    """
    Calculate the absolute coordinates of the end edge point of the obstacle
    """
    def edgeCoordinates(self, filtered_row):
        _estimated_kinematics = self.estimated_kinematics
        if filtered_row == []:
            return None
        else:
            endpoint = filtered_row[-1]

            x_pos, y_pos, z_pos = (_estimated_kinematics.position)
            edge = [x_pos + endpoint[0], y_pos + endpoint[1], z_pos + endpoint[2]]
            # print("Edge COORD: ", edge)


    #function to get the distance between two points
    def distanceTwoPoints(self, point1, point2):
        temp = point1 - point2
        dist = np.sqrt(temp[0]**2 + temp[1]**2)
        return dist

    """
    Description: Iterative check for simple path following from a list of coordinates
    """
    def checkIfAtWaypt(self, point):
        #Todo: add some margin so it doesn't have to be at the exact coordinates of the goal
        x_pos, y_pos, z_pos = self.estimated_kinematics.position

        # xtol = math.isclose(x_pos, point[0], rel_tol=0.5)
        # ytol = math.isclose(y_pos, point[1], rel_tol=0.5)

        # if (xtol and ytol):
        #     return True
        # else:
        #     return False
        
        tol = self.distanceTwoPoints(np.array([x_pos, y_pos]), np.array([point[0], point[1]]))
        if tol < 0.2:
            return True
        else:
            return False
    
    def spawnObjects(self):
        vclient = airsim.VehicleClient()
        vclient.confirmConnection()

        all_objects = vclient.simListSceneObjects()
        print(f"Objects: {all_objects}")
        # print("object torus5 ", all_objects[18])
        # assets = self.client.simListAssets()
        # print(f"Assets: {assets}")
        
        scale = airsim.Vector3r(1.0, 1.0, 1.0)
        
        asset_name = 'SM_MERGED_TriggerVolume7'
        desired_name = f"{asset_name}_spawn_{random.randint(0, 100)}"
        
        poses = [[5,0.1,0],[15,8,0], [27,8,0], [38, -4, 0], [50, 1,0]]
        for pose in poses:
            # pose = airsim.Pose(position_val=airsim.Vector3r(7.0, 0.0, 0.0))
            # pose = vclient.simGetObjectPose(all_objects[18])
            input_pose = airsim.Pose(position_val=airsim.Vector3r(pose[0], pose[1], pose[2]))
        
            # obj_name = self.client.simSpawnObject(object_name=desired_name, asset_name=asset_name, pose=pose, scale=scale)
            obj_name = vclient.simSpawnObject(desired_name, asset_name, input_pose, scale, False, False)
            print(f"Created object {desired_name} from asset {asset_name} at pose {pose}, scale {scale}")
        
        
        
        

    def execute(self):
        
        self.spawnObjects()
        
        print("arming the drone...")

        self.client.armDisarm(True)

        self.takeoff()
        

        airsim.wait_key('Press any key to lift drone')
        self.client.moveToPositionAsync(0, 0, -1, 5).join()

        # waypoint = [[5,5,0],[10,10,0],[30,23,0]]
        waypoint = [[5,0.1,0],[15,8,0], [27,8,0], [38, -4, 0], [50, 1,0]]
        n = 0
        try:
            while waypoint != []:
                self.client.moveToPositionAsync(waypoint[n][0],waypoint[n][1],waypoint[n][2], 3).join()

                x_pos, y_pos, z_pos = self.estimated_kinematics.position
                print("Current Position: ", x_pos, y_pos)
                print("Waypoint: ", waypoint[n])
                
                if (self.checkIfAtWaypt(waypoint[n])):
                    
                    print("Current reached Position: ", x_pos, y_pos)
                    waypoint.pop(0)

                    print('REACHED')

            print(waypoint)    
            self.client.moveByVelocityAsync(0, 0, 0, 5).join()    
            # arrived at goal
            self.stop()
            print('Reached Goal')


        except KeyboardInterrupt:
            self.stop()





if __name__ == "__main__":
    print('Main accessed')
    args = sys.argv
    args.pop(0)

    arg_parser = argparse.ArgumentParser("Lidar.py makes drone fly and gets Lidar data")

    arg_parser.add_argument('-save-to-disk', type=bool, help="save Lidar data to disk", default=False)


    args = arg_parser.parse_args(args) 
    ssoa = ClearPathObstacleAvoidance()
    try:
        ssoa.execute()
    except KeyboardInterrupt:
        ssoa.stop()

    finally:
        # lidarTest.display_lidar()
        ssoa.stop()