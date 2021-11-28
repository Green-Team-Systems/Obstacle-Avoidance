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

# Makes the drone fly and get Lidar data

class ObstacleAvoidance:
    
    destination = (-100, 0, 0)
    collisiondist = 4
    isTurning = False
    collision = False

    @property
    def estimated_kinematics(self):
        return self.client.getMultirotorState().kinematics_estimated


    def __init__(self):

        # connect to the AirSim simulator
        
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

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

    def scan(self,vehicle_name,lidar_names):
        # print('Scanning ...')
        # Scan lidar data
        next_row = []
        y_points_last = 0
        overall_point_list=[]
        for lidar_name in lidar_names:

            lidar_data = self.client.getLidarData(lidar_name=lidar_name,vehicle_name=vehicle_name)
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
    Description: Turn drone towards goal asynchronous
    
    Inputs: vehicle_name, lidar_names
    
    Outputs: 
    
    Notes: Need to fix function to save coordinates in class

    """

    def turnTowardsGoal(self, vehicle_name, lidar_names):
        print('turning towards goal')
        dist = math.sqrt((self.destination[0]**2)+(self.destination[1]**2))
        #theta = math.acos()
        rotate = True
        isTurning = True
        
        length = 5
        kinematicsEstimated = self.client.getMultirotorState().kinematics_estimated
        w_val, x_val, y_val, z_val = (kinematicsEstimated.orientation)
        x_pos, y_pos, z_pos = (kinematicsEstimated.position)
        roll_x, pitch_y, yaw_z = self.euler_from_quaternion(x_val, y_val, z_val, w_val) #yaw_z is the global radian angle of the drone
        points = [x_pos + length * math.cos(yaw_z), y_pos + length * math.sin(yaw_z)]
        # gps = self.client.getLidarData(lidar_name=lidar_names[0],vehicle_name=vehicle_name).pose.position
        # print(gps)

        u1 = self.destination[0] - x_pos
        u2 = self.destination[1] - y_pos

        v1 = points[0] - x_pos
        v2 = points[1] - y_pos

        angleInRad = self.angle_of_vectors((u1,u2), (v1,v2))
        angleInDegrees = math.degrees(angleInRad)

        # angle = self.angle_of_vectors()
        #self.client.moveByRollPitchYawrateThrottleAsync(0, 0, 0, 0.6, 0.3, vehicle_name=vehicle_name).join()
        self.client.rotateToYawAsync(angleInDegrees, timeout_sec=0.1, margin=0.1, vehicle_name=vehicle_name)
        return angleInRad, angleInDegrees
        
    """
    Description: Turn drone towards goal synchronous
    
    Inputs: vehicle_name, lidar_names
    
    Outputs: 
    
    Notes: Need to fix function to save coordinates in class

    """

    def turnTowardsGoalSynchronous(self, vehicle_name, lidar_names):
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
        # gps = self.client.getLidarData(lidar_name=lidar_names[0],vehicle_name=vehicle_name).pose.position
        # print(gps)

        u1 = self.destination[0] - x_pos
        u2 = self.destination[1] - y_pos

        v1 = points[0] - x_pos
        v2 = points[1] - y_pos

        angleInRad = self.angle_of_vectors((u1,u2), (v1,v2))
        angleInDegrees = math.degrees(angleInRad)

        # angle = self.angle_of_vectors()
        #self.client.moveByRollPitchYawrateThrottleAsync(0, 0, 0, 0.6, 0.3, vehicle_name=vehicle_name).join()
        # print('started turning towards goal')
        self.client.rotateToYawAsync(angleInDegrees, timeout_sec=10, margin=0.1, vehicle_name=vehicle_name).join()
        # print('finished turning towards goal')
        return angleInRad, angleInDegrees

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



    def vision(self, overall_point_list):
      
        top_row = overall_point_list[1] # or 0?
        midpoint_top_level_ind = int(len(overall_point_list[1]) / 2)
        x_val = overall_point_list[1][midpoint_top_level_ind][0]
        y_val = overall_point_list[1][midpoint_top_level_ind][1]
        z_val = overall_point_list[1][midpoint_top_level_ind][2]

        return x_val, y_val, z_val

    def checkForCollision(self, collision_distance, overall_point_list):
        #stop = False
        
        x_val, y_val, z_val = self.vision(overall_point_list)
        # print(f"x val: {x_val}" )
        # print(f"y val: {y_val}" )
        # print(f"z val: {z_val}" )
        if( x_val <= collision_distance):
            # self.stopDrone()
            return True
        return False


    def stop(self):

        airsim.wait_key('Press any key to reset to original state')

        self.client.armDisarm(False)
        self.client.reset()

        self.client.enableApiControl(False)
        print("Done!\n")

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
    Description: Produce a array of boolean values for each point in the row based on distance threshold
    
    Inputs: view_distance, overall_point_list, row_index
    
    Outputs: boolean array
    
    Notes: Used to find the obstacles location in relation to the drones FOV

    """

    def booleanArray_parser(self, view_distance, overall_point_list, row_index):
        booleanArray = list()
        midline = overall_point_list[row_index]

        #trim midline to set 0s in boolean array where any pairs have x > 5
        #trim y-values to only look at middle 50 points

        # Array construction
        x_list = []
        for xval,yval,zval in midline:
            x_list.append(xval)
        
        
        for x in x_list:
            if x < view_distance: # test narrower and shorter narrowview
                booleanArray.append(1)
            else:
                # print ("appended 0")
                booleanArray.append(0)
        
        midpoint_booleanArray_ind = int(len(booleanArray)/2) # Mid index

        leftHalf_booleanArray = booleanArray[0:midpoint_booleanArray_ind]
        rightHalf_booleanArray = booleanArray[midpoint_booleanArray_ind:len(booleanArray)]

        # middle n points
        offset = 50
        narrow_view = booleanArray[midpoint_booleanArray_ind-offset:midpoint_booleanArray_ind+offset]
        narrow_view = np.array(narrow_view)

        return booleanArray, leftHalf_booleanArray, rightHalf_booleanArray, narrow_view


    def split_list(self,thelist, delimiter):
        ''' Split a list into sub lists, depending on a delimiter.

        delimiters - item or tuple of item
        '''
        results = []
        sublist = []
        zeros = []
        zero_sublist = []

        for item in thelist:
            if (item == delimiter):
                if (len(sublist) > 0):
                    print (sublist)
                    results.append(sublist) # old one
                zero_sublist.append(item)
                
                sublist = []            # new one
            else:
                if (len(zero_sublist) > 0):
                    print (zero_sublist)
                    zeros.append(zero_sublist)
                sublist.append(item)
                
                zero_sublist = []

        if sublist:  # last bit
            results.append(sublist)
            print (sublist)

        if zero_sublist:
            zeros.append(zero_sublist)
            print (zero_sublist)


        return results



    def wall_drawer(self, booleanArray, chosen_row):
        # print([list(j) for i, j in groupby(booleanArray)])
        self.split_list(booleanArray, 0)

    """
    Description: detects gaps between points and returns row of lidar points with a gap symbol
    
    Inputs: threshold - distance between points, row of points
    
    Outputs: Fixed row of points
    
    Notes:

    """

    def dataFixer(self, threshold, chosen_row):

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

        return chosen_row
    
    def view_distance_filter(self, view_distance, fixed_row):

        #Todo: make function consider 'G'
        filtered_row = []
        for ind, val in enumerate(fixed_row):
            
            if val != 'G':
                dist = val[0]
                if(dist <= view_distance):
                    filtered_row.append(val)

        return filtered_row
        

    def calculate_slope(self, fixed_row):
        #find left-most chunk of points between gaps

        first_point = fixed_row[0]
        last_point = fixed_row[0]
        for ind, point in enumerate(fixed_row):
            #this will break if a G is at the start of the row
            if point == 'G':
                last_point = fixed_row[ind - 1]
            last_point = fixed_row[ind]
        
        slope = (last_point[1] - first_point[1]) / (last_point[0] - first_point[0])
        #y = slope * x + firstpoint[1]  
        return slope

    # """
    # Description:
    
    # Inputs:
    
    # Outputs:
    
    # Notes:

    # """        
    # def estimate_object_vector(self, fixed_row):
    #     #find left-most chunk of points between gaps

    #     first_point = fixed_row[0]
    #     last_point = fixed_row[0]
    #     for ind, point in enumerate(fixed_row):
    #         #this will break if a G is at the start of the row
    #         if point == 'G':
    #             last_point = fixed_row[ind - 1]
    #             break
    #         last_point = fixed_row[ind]
        
    #     vector = ((last_point[0] - first_point[0]), (last_point[1] - first_point[1]))
    #     return vector

    """
    Description: Calculate the sum of the vectors from left to right
    
    Inputs: row of lidar points 
    
    Outputs: sum vector
    
    Notes:

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


    def follow_vector(self, vector, velocity, vehicle_name):
        
        #follow parallel to obstacle (strafe)
        self.client.moveToPositionAsync(vector[0], vector[1], 0, velocity, 0.5, vehicle_name=vehicle_name)
        # self.client.moveByVelocityAsync(vector[0], vector[1], vector[2]

        return

    """
    Description: Move forward until the lidar detects an obstacle
    
    Inputs: vehicle_name, lidar name
    
    Outputs: None
    
    Notes:

    """

    def approach(self, vehicle_name, lidar_names):
        collisioncheck = False
        shouldMove = True
        try:
            while shouldMove==True:
                
                overall_point_list = self.scan(vehicle_name,lidar_names)
                self.client.moveByVelocityBodyFrameAsync(3, 0, 0,0.1)
                
                collisioncheck = self.checkForCollision(5, overall_point_list)

                # exit when collision into obstacle
                if(collisioncheck == True):
                    self.stopDrone()
                    
                    shouldMove = False
                    
        except KeyboardInterrupt:
            airsim.wait_key('Press any key to stop running this script')
            lidarTest.stop()
            print("Done!\n")

    def normalize_vector(self, vector):
        

        try:
            vx = vector[0] / math.sqrt(vector[0]**2 + vector[1]**2)
            vy = vector[1] / math.sqrt(vector[0]**2 + vector[1]**2)
            normalized_vector = [vx, vy]
        except ZeroDivisionError:
            return 0

        return normalized_vector

    """
    Description: Calculates the offset angle (45 deg) from the wall vector
    
    Inputs: wall vector
    
    Outputs: Offset wall vector
    
    Notes:

    """

    def vector_from_wall(self, wall_vector):
        #rotates vector by 45 degrees
        offwallvector = [(math.cos(math.sqrt(3)/2) * wall_vector[0]) - (math.sin(1/2) * wall_vector[1]), (math.sin(math.sqrt(2)/2) * wall_vector[0]) + (math.cos(math.sqrt(2)/2)* wall_vector[1])]
        return offwallvector

    """
    Description: Calculate the angle from the drone orientation vector to the input vector
    
    Inputs: objective vector
    
    Outputs: angle in degrees
    
    Notes:

    """

    def angle_from_drone_to_vector(self, vector, vehicle_name, lidar_names):
        print('turning towards goal')
        
        
        length = 5
        _estimated_kinematics = self.estimated_kinematics
        w_val, x_val, y_val, z_val = (_estimated_kinematics.orientation)
        x_pos, y_pos, z_pos = (_estimated_kinematics.position)
        roll_x, pitch_y, yaw_z = self.euler_from_quaternion(x_val, y_val, z_val, w_val) #yaw_z is the global radian angle of the drone
        points = [x_pos + length * math.cos(yaw_z), y_pos + length * math.sin(yaw_z)]
        # gps = self.client.getLidarData(lidar_name=lidar_names[0],vehicle_name=vehicle_name).pose.position
        # print(gps)

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
    
    Notes:

    """

    def angle_of_vectors(self, vector1, vector2):
        [a, b] = vector1
        [c, d] = vector2

        dotProduct = a*c + b*d
            # for three dimensional simply add dotProduct = a*c + b*d  + e*f 
        modOfVector1 = math.sqrt( a*a + b*b)*math.sqrt(c*c + d*d) 
            # for three dimensional simply add modOfVector = math.sqrt( a*a + b*b + e*e)*math.sqrt(c*c + d*d +f*f) 
        
        

        angle = dotProduct/modOfVector1
        #  print("Cosθ =",angle)
        # angleInDegree = math.degrees(math.acos(angle))
        angleInRad = math.acos(angle)
        #  print("θ =",angleInDegree,"°")

        return angleInRad

    def stopDrone(self):
        print('Stopping')
        self.client.moveByVelocityAsync(0, 0, 0, 2).join()
        
    """
    Description:
    
    Inputs:
    
    Outputs:
    
    Notes:

    """
    def execute(self,vehicle_name,lidar_names):
        print("arming the drone...")
        
        self.client.armDisarm(True)

        self.takeoff()  
        
        airsim.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
        # self.client.moveToPositionAsync(20, 0, 0, 5).join()
        self.client.moveToPositionAsync(0, 0, -1, 5).join()

        print('Scanning Has Started\n')
        print('Use Keyboard Interrupt \'CTRL + C\' to Stop Scanning\n')
        # [Go Forward, Turn Right, Turn Left]

        Armed = True
        

        try:
            self.turnTowardsGoalSynchronous(vehicle_name,lidar_names)
            #approach to within 5 meters of wall
            self.approach(vehicle_name, lidar_names)
            #turn lidar data into list
            overall_point_list = self.scan(vehicle_name,lidar_names)
            #choose the row nearest to z = 0 (relative to drone)
            chosenRowIndex = self.chooseRow(overall_point_list)
            #correct for gaps in data (if no wall is behind, lidar will omit any gaps)
            fixedchosenRow = self.dataFixer(1, overall_point_list[chosenRowIndex])
            
            filtered_row = self.view_distance_filter(5, fixedchosenRow)

            # get vector 45 degrees from wall 
            vectorfromwall = self.vector_from_wall(self.calculate_object_sum_vector(filtered_row))
            print("Vector from wall: " + str(vectorfromwall))
            # get angle from drone to wall vector
            angleInRad,angleInDegree = self.angle_from_drone_to_vector(vectorfromwall, vehicle_name, lidar_names)
            print("Turning angle: " + str(angleInDegree))
            self.client.rotateToYawAsync(angleInDegree, timeout_sec=30, margin=0.1, vehicle_name=vehicle_name).join()

            startTime = time.time()
            while((time.time() - startTime) <= 100):
                try:
                    overall_point_list = self.scan(vehicle_name,lidar_names)
                    chosenRowIndex = self.chooseRow(overall_point_list)
                    fixedchosenRow = self.dataFixer(1, overall_point_list[chosenRowIndex])
                    filtered_row = self.view_distance_filter(5, fixedchosenRow)

                    if (filtered_row == []):
                        print("No wall in view")
                        self.turnTowardsGoalSynchronous(vehicle_name,lidar_names)
                        self.approach(vehicle_name, lidar_names)
                    else:
                        print('following vector')
                        self.follow_vector(self.calculate_object_sum_vector(filtered_row), 5, vehicle_name)
                        vectorfromwall = self.vector_from_wall(self.calculate_object_sum_vector(fixedchosenRow))
                        print("Vector from wall: " + str(vectorfromwall))   
                        angleInDegree = self.angle_from_drone_to_vector(vectorfromwall, vehicle_name, lidar_names)[1]
                        print("Wall turning angle: " + str(angleInDegree))
                        self.client.rotateToYawAsync(angleInDegree, timeout_sec=0.5, margin=0.1, vehicle_name=vehicle_name)
                    
                except KeyboardInterrupt:
                    lidarTest.stop()
                    break

                

        except KeyboardInterrupt:
            self.stop()

                
if __name__ == "__main__":
    print('Main accessed')
    args = sys.argv
    args.pop(0)

    arg_parser = argparse.ArgumentParser("Lidar.py makes drone fly and gets Lidar data")

    arg_parser.add_argument('-save-to-disk', type=bool, help="save Lidar data to disk", default=False)
    vehicle_name,lidar_names = 'Drone1',['LidarSensor1']
  
    args = arg_parser.parse_args(args) 
    lidarTest = ObstacleAvoidance()
    try:
        lidarTest.execute(vehicle_name,lidar_names)
    except KeyboardInterrupt:
        lidarTest.stop()
          
    finally:
        # lidarTest.display_lidar()
        lidarTest.stop()

