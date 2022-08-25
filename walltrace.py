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

from itertools import groupby


# Wall trend tracker

# take in points of xy and return slope and intercept
# offset from the wall by d distance and draw trend line
# follows trend line until 'wallrun' is false (left end is cleared)
# check if can go to goal, if not snap 45 degrees to left
# continue back to going right (relative to the fov window) until wallrun is true

class WallTrace:
    
    destination = (-100, 50, 0)
    collisiondist = 4
    vehicle_name,lidar_names = 'Drone1',['LidarSensor1']

    #TODO: could use enum
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

    """
    Description: This function is used to get the lidar point list data from the drone.
    
    Inputs:vehicle name,lidar names
    
    Outputs: 2d Array of 3d Lidar Data
    
    Notes: Array is grouped by rows, each row is a list of 3d points
    structured as follows - [[[x,y,z],[x,y,z],[x,y,z]], [[x,y,z],[x,y,z],[x,y,z]], [[x,y,z],[x,y,z],[x,y,z]]]
    """

    def scan(self, lidar_data):
        next_row = []
        y_points_last = 0
        overall_point_list=[]
        for lidar_name in self.lidar_names:
            for i in range(0, len(lidar_data), 3):
                xyz = lidar_data[i:i+3]
                # Check at the end of each row for positive y and negative y value at the beginning of new row
                if (xyz[1] != math.fabs(xyz[1]) and y_points_last == math.fabs(y_points_last)):
                    overall_point_list.append(next_row)
                    next_row = list()
                next_row.append(xyz)
                y_points_last = xyz[1]
        return overall_point_list


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

    """
    Description: Find closest row parallel to the z level of the drone
    
    Inputs: overall_point_list
    
    Outputs: row index i
    
    Notes:
    """

    def chooseRow(self, overall_point_list):
        top_row = overall_point_list[1] 
        mid_point_top_level_ind = int(len(top_row) / 2)
        z_level_row = top_row[mid_point_top_level_ind][2] # initial z val of top row mid
        i = 1

        for row_index, row in enumerate(overall_point_list):
            if(len(row) > 0):
                mid_point = int(len(row) / 2) # index of mid point
                mid_point_z = row[mid_point][2] # z value of mid point
                if(abs(mid_point_z) < abs(z_level_row)):
                    z_level_row = mid_point_z

                    i = row_index

        return i


    """
    Description: detects gaps between points and returns row of lidar points with a gap symbol
    
    Inputs: threshold - distance between points, row of points
    
    Outputs: Fixed row of points

    """
    def dataFixer(self, threshold, chosen_row):

        #Todo: place 'Gap' at the end / corner
        last_point = np.array([chosen_row[0][0], chosen_row[0][1]])  #list of x and y value of first point
        for ind, point_val in enumerate(chosen_row):
            temp = last_point - np.array([point_val[0], point_val[1]])
            vector_dist = np.sqrt(temp[0]**2 + temp[1]**2)

            #calculated the vector distance between two points to see if there's a gap in the lidar data
            if vector_dist > threshold:
                chosen_row.insert(ind, 'G')
            last_point = np.array([point_val[0], point_val[1]])

        return chosen_row
    
    """
    Description: Filter lidar data by a distance threshold
    
    Inputs: view distance set, row of lidar points
    
    Outputs: filtered row of lidar points
    """
    def view_distance_filter(self, view_distance, fixed_row):
        #Todo: make function consider 'G'
        filtered_row = []
        for ind, val in enumerate(fixed_row):
            if val != 'G':
                dist = val[0]
                if(dist <= view_distance):
                    filtered_row.append(val)


        return filtered_row
     
    """
    Description: Calculate the sum of the vectors from left to right
    
    Inputs: row of lidar points 
    
    Outputs: sum vector
    """
    def calculate_object_sum_vector(self, fixed_row):
        #find left-most chunk of points between gaps

        first_point = fixed_row[0]
        sum_vector = [0, 0]
        for ind, point in enumerate(fixed_row):
            #this will break if a G is at the start of the row
            if point == 'G':
                break
            
            vector = [(point[0] - first_point[0]), (point[1] - first_point[1])]
            sum_vector = [sum_vector[0] + vector[0], sum_vector[1] + vector[1]]
            
        return sum_vector

    def normalize_vector(self, vector):
        
        try:
            vx = vector[0] / math.sqrt(vector[0]**2 + vector[1]**2)
            vy = vector[1] / math.sqrt(vector[0]**2 + vector[1]**2)
            normalized_vector = [vx, vy]
        except ZeroDivisionError:
            return [0,0]

        return normalized_vector

    """
    Description: Calculates the offset angle (45 deg) from the wall vector
    
    Inputs: wall vector
    
    Outputs: Offset wall vector
    """
    def vector_45_from_wall(self, wall_vector):
        #rotates vector by 45 degrees
        x_component = float(-1) * (math.cos(math.sqrt(2)/2) * wall_vector[0]) - (math.sin(math.sqrt(2)/2) * wall_vector[1])
        y_component = (math.sin(math.sqrt(2)/2) * wall_vector[0]) + (math.cos(math.sqrt(2)/2)* wall_vector[1])
        offwallvector = [x_component, y_component]
        
        return offwallvector

    """
    Description: Calculate the angle from the drone orientation vector to the input vector
    
    Inputs: objective vector
    
    Outputs: angle in degrees
    """
    def angle_from_drone_to_vector(self, vector):

        length = 5
        _estimated_kinematics = self.estimated_kinematics
        w_val, x_val, y_val, z_val = (_estimated_kinematics.orientation)
        x_pos, y_pos, z_pos = (_estimated_kinematics.position)
        roll_x, pitch_y, yaw_z = self.euler_from_quaternion(x_val, y_val, z_val, w_val) #yaw_z is the global radian angle of the drone
        points = [x_pos + length * math.cos(yaw_z), y_pos + length * math.sin(yaw_z)]

        # u1 = self.destination[0] - x_pos
        # u2 = self.destination[1] - y_pos

        u1 = vector[0]
        u2 = vector[1]
        
        [u1,u2] = self.normalize_vector([u1, u2])

        v1 = points[0] - x_pos
        v2 = points[1] - y_pos

        [v1,v2] = self.normalize_vector([v1, v2])

        angleInRad = self.angle_of_vectors((u1,u2), (v1,v2))
        angleInDegrees = math.degrees(angleInRad)
        
        return angleInRad, angleInDegrees


    """
    Description: Takes two vectors and returns the angle between them
    
    Inputs: vector1, vector2
    
    Outputs: angle (radians)
    """
    def angle_of_vectors(self, vector1, vector2):
        [a, b] = vector1
        [c, d] = vector2

        dotProduct = a*c + b*d
        # for three dimensional add dotProduct = a*c + b*d  + e*f 
        modOfVector1 = math.sqrt( a*a + b*b)*math.sqrt(c*c + d*d) 
        # for three dimensional add modOfVector = math.sqrt( a*a + b*b + e*e)*math.sqrt(c*c + d*d +f*f) 
        
        if (modOfVector1 == 0):
            return 0
        angle = dotProduct/modOfVector1
        # angleInDegree = math.degrees(math.acos(angle))
        angleInRad = math.acos(angle)
        #  print("θ =",angleInDegree,"°")

        return angleInRad

    # Avoidance functions
    def avoid(self, lidar_data):
        # turn 45 against wall
        # strafe with wall vector
        # break out when filtered view is clear

        overall_point_list = self.scan(lidar_data)
        #choose the row nearest to z = 0 (relative to drone)
        chosenRowIndex = self.chooseRow(overall_point_list)
        #correct for gaps in data (if no wall is behind, lidar will omit any gaps)
        fixedchosenRow = self.dataFixer(1, overall_point_list[chosenRowIndex])
        
        filtered_row = self.view_distance_filter(5, fixedchosenRow)

        # get vector 45 degrees from wall
        if (filtered_row is None):
            return
        sum_vector = self.calculate_object_sum_vector(fixedchosenRow)
        norm_sum_vector = self.normalize_vector(sum_vector)
        if(norm_sum_vector is [0,0]):
            return
        vectorfromwall = self.vector_45_from_wall(norm_sum_vector)
        # get angle from drone to wall vector
        angleInRad,angleInDegree = self.angle_from_drone_to_vector(vectorfromwall)

        # TODO: how to pass yaw_mode parameter
        x_Vel = norm_sum_vector[0]
        y_Vel = norm_sum_vector[1]
        z_Vel = 0
        

        return angleInDegree, x_Vel, y_Vel, z_Vel


    # Main execution loop for mode switch and drone movement
    def execute(self, lidar_data):
        at_Goal = False
        #get lidar
        #turn lidar data into list
        overall_point_list = self.scan(lidar_data)
        #choose the row nearest to z = 0 (relative to drone level)
        chosenRowIndex = self.chooseRow(overall_point_list)
        #correct for gaps in data (if no wall is behind, lidar will omit any gaps)
        fixedchosenRow = self.dataFixer(1, overall_point_list[chosenRowIndex])
        # filter lidar array to get only the points within a certain distance
        filtered_row = self.view_distance_filter(5, fixedchosenRow)
        
        # Flight mode switch
        if (filtered_row != [] and self.mode == self.STATES["Clear"]):
            self.mode = self.STATES["Obstacle"]
        elif (filtered_row == [] and self.mode == self.STATES["Obstacle"]):
            self.mode = self.STATES["Clear"]
        
        # execution of the current flight mode
        if self.mode == "TOGOAL":
            # turn to destination coordinates on the xy plane
            x_Vel = 0
            y_Vel = 0
            angleInDegree = 0
        elif self.mode == "AVOID":
            angleInDegree, x_Vel, y_Vel, z_Vel = self.avoid(lidar_data)

        # return angle in degrees, x_Vel, y_Vel, z_Vel
        return angleInDegree, x_Vel, y_Vel, z_Vel
            
    def getGoalAngle(self):
        print('turning towards goal')
        dist = math.sqrt((self.destination[0]**2)+(self.destination[1]**2))
        #theta = math.acos()
        rotate = True
        isTurning = True

        # while rotate == True:
        #airsim.types.EnvironmentState.position
        # print(self.client.getMultirotorState().orientation)
        
        length = 5
        # getter method updates this value
        _estimated_kinematics = self.estimated_kinematics
        w_val, x_val, y_val, z_val = (_estimated_kinematics.orientation)
        x_pos, y_pos, z_pos = (_estimated_kinematics.position)
        roll_x, pitch_y, yaw_z = self.euler_from_quaternion(x_val, y_val, z_val, w_val) #yaw_z is the global radian angle of the drone
        points = [x_pos + length * math.cos(yaw_z), y_pos + length * math.sin(yaw_z)]
        # gps = self.client.getLidarData(lidar_name=self.lidar_names[0],self.vehicle_name=self.vehicle_name).pose.position
        # print(gps)

        u1 = self.destination[0] - x_pos
        u2 = self.destination[1] - y_pos

        v1 = points[0] - x_pos
        v2 = points[1] - y_pos

        angleInRad = self.angle_of_vectors((u1,u2), (v1,v2))
        angleInDegrees = math.degrees(angleInRad)

        return angleInRad, angleInDegrees

    def goToGoal(self):
        #tuple[1] to get degrees
        goalAngle = self.getGoalAngle()[1]
        self.client.moveByVelocityBodyFrameAsync(3, 0, 0, 0.3, yaw_mode = airsim.YawMode(False, goalAngle))

