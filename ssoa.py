from sqlite3 import Date
from tracemalloc import start
from matplotlib import projections
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

import matplotlib.pyplot as plt
import numpy as np
import math as m

from itertools import groupby
from datetime import datetime

from airsim.types import YawMode
from plot_ssoa import run_plot
from multiprocessing import Queue
from utils.data_classes import PosVec3
from utils.position_utils import position_to_list
from path_planning import *

# OA algorithm that generates new way points based on safety sphere around obstacles

class ClearPathObstacleAvoidance:
    
    destination = (-100, 50, 0)
    collisiondist = 4
    drone_id = "Drone1"
    path_planning_queue = Queue(20)
    vehicle_name,lidar_names = 'Drone1',['LidarSensor1']
    pid = PathPlanning(
        path_planning_queue,
        drone_id,
        simulation=True
    )
    
    

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
                # if (xyz[1] != math.fabs(xyz[1]) and y_points_last == math.fabs(y_points_last)):
                #     overall_point_list.append(next_row)
                #     next_row = list()
                # next_row.append(xyz)
                # y_points_last = xyz[1]
                overall_point_list.append(xyz)
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
        
        return edge
            
        
    """
    Description: Iterative check for simple path following from a list of coordinates
    """
    def checkIfAtWaypt(self, point):
        #Todo: add some margin so it doesn't have to be at the exact coordinates of the goal
        x_pos, y_pos, z_pos = self.estimated_kinematics.position
        
        xtol = math.isclose(x_pos, point[0], rel_tol=0.5)
        ytol = math.isclose(y_pos, point[1], rel_tol=0.5)
        
        if (xtol and ytol):
            return True
        else:
            return False
    
    
    def FrameVector(self):
        length = 5
        kinematicsEstimated = self.client.getMultirotorState().kinematics_estimated
        w_val, x_val, y_val, z_val = (kinematicsEstimated.orientation)
        x_pos, y_pos, z_pos = (kinematicsEstimated.position)
        roll_x, pitch_y, yaw_z = self.euler_from_quaternion(x_val, y_val, z_val, w_val) #yaw_z is the global radian angle of the drone
        points = [x_pos + length * math.cos(yaw_z), y_pos + length * math.sin(yaw_z)]
        
        v1 = points[0] - x_pos
        v2 = points[1] - y_pos
        
        # print("v1:", v1, " v2:", v2)
        
        return v1, v2
        
    """
    Description: Convert a quaternion into euler angles (roll, pitch, yaw)
    
    Inputs: 
    
    Outputs:Orientation in Euler radian roll_x, pitch_y, yaw_z
    
    Notes:
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)

    """

    def euler_from_quaternion(self,x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians               
        
    def sphere_intersection(self, edgepoint, vector, radius):
        # ss = (x - edge_x)^2 + (y  - edge_y)^2
        # (y - y_pos) = (vector_x / vector_y) * (x - x_pos)
        
        
        _estimated_kinematics = self.estimated_kinematics
        x_pos, y_pos, z_pos = self.estimated_kinematics.position
        # negative x to account for euclidean space config 
        if vector[0] < 0:
            vector[0] = 0.01
        elif vector[1] < 0:
            vector[1] = 0.01
        slope = -vector[0] / vector[1]
        
        z = (-(slope) * x_pos) + y_pos - edgepoint[1]
        a = 1 + math.pow(vector[0]/ vector[1], 2)
        b = -2 * edgepoint[0] + (2 * (slope) * z)
        c = math.pow(z, 2) + math.pow(edgepoint[0], 2)
        
        print("slope", slope)
        print("Edge Coord",edgepoint)
        print("a:", a, " b:", b, " c:", c, " z:", z)
        
        #quadratic formula
        x1 = (-b + math.sqrt(math.pow(b, 2) - 4 * a * c)) / (2 * a)
        x2 = (-b - math.sqrt(math.pow(b, 2) - 4 * a * c)) / (2 * a)
        
        y1 = math.sqrt(math.pow(radius, 2) - math.pow(x1 - edgepoint[0], 2)) + edgepoint[1]
        y2 = math.sqrt(math.pow(radius, 2) - math.pow(x2 - edgepoint[0], 2)) + edgepoint[1]
        
    
    def get_vector(self, pos, waypt):
        vec = [waypt[0] - pos[0], waypt[1] - pos[1], waypt[2] - pos[2]]
        
        return vec
    
    def execute(self):
        print("arming the drone...")
        self.client.armDisarm(True)

        self.takeoff()        
        airsim.wait_key('Press any key to lift drone')
        state = self.client.getMultirotorState()
                
        
        starting_pos = position_to_list(state.kinematics_estimated.position)
        #self.client.moveToPositionAsync(0, 0, -1, 5).join()
        
        waypoint = [[starting_pos.X,starting_pos.Y,starting_pos.Z],[70, 0, 5]]
        # waypoint = [[5,0,0], [7,0,0]]
        #obstacle dimenesions are 40x40x10
        new_command = 0
        new_waypoint = PosVec3()
        #position = position_to_list(state.kinematics_estimated.position, starting_position=starting_pos, frame="global")
        #turn lidar data into list
        startTime = datetime.utcnow()
        current_pos = position_to_list(state.kinematics_estimated.position)
        dist_treshold = 2.0
        i = 0
        previous_waypoint_len = len(waypoint)
        while current_pos.X < 70:
            overall_point_list = self.scan()
            for x in overall_point_list:
                x.append(1)
            run_plot(waypoint, overall_point_list, True)
            if(len(waypoint) != previous_waypoint_len):
                self.client.simPause(True)
                run_plot(waypoint, overall_point_list, True)
                self.client.simPause(False)
            state = self.client.getMultirotorState()
            current_pos = position_to_list(state.kinematics_estimated.position)
            dist_to_target = ned_position_difference(current_pos, new_waypoint)
            if dist_to_target < dist_treshold:
                i = i + 1
            print(i)
            new_waypoint.X = waypoint[i][0]
            new_waypoint.Y = -waypoint[i][1]
            new_waypoint.Z = -waypoint[i][2]
            print(f'Way point : {new_waypoint}')
            x_Vel, y_Vel, z_Vel = self.pid.calculate_velocities(
            new_waypoint,
            current_pos, 
            startTime
            )
            heading = np.arctan2(new_waypoint.Y,new_waypoint.X)
            heading = m.degrees(heading)
            self.client.moveByVelocityAsync(
            x_Vel,
            y_Vel,
            z_Vel,
            m.inf,
            yaw_mode=YawMode(False,heading)
            )
            startTime = datetime.utcnow()
            previous_waypoint_len = len(waypoint)
            print(current_pos)
        
if __name__ == "__main__":
    
    print('Main accessed')

    ssoa = ClearPathObstacleAvoidance()
    
    try:
        ssoa.execute()
    except KeyboardInterrupt:
        ssoa.stop()
          
    finally:
        # lidarTest.display_lidar()
        ssoa.stop()

