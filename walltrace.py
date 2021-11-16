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
class LidarTest:
    
    destination = (100,0,0)
    collisiondist = 4
    isTurning = False
    collision = False

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

    def turnTowardsGoal(self, vehicle_name, lidar_names):
        print('turning towards goal')
        self.client.hoverAsync().join()
        dist = math.sqrt((self.destination[0]**2)+(self.destination[1]**2))
        #theta = math.acos()
        rotate = True
        isTurning = True

        # while rotate == True:
        #airsim.types.EnvironmentState.position
        # print(self.client.getMultirotorState().orientation)
        
        length = 5
        w_val, x_val, y_val, z_val = (self.client.getMultirotorState().kinematics_estimated.orientation)
        x_pos, y_pos, z_pos = (self.client.getMultirotorState().kinematics_estimated.position)
        roll_x, pitch_y, yaw_z = self.euler_from_quaternion(x_val, y_val, z_val, w_val) #yaw_z is the global radian angle of the drone
        points = [x_pos + length * math.cos(yaw_z), y_pos + length * math.sin(yaw_z)]
        # gps = self.client.getLidarData(lidar_name=lidar_names[0],vehicle_name=vehicle_name).pose.position
        # print(gps)

        u1 = self.destination[0] - x_pos
        u2 = self.destination[1] - y_pos

        v1 = points[0] - x_pos
        v2 = points[1] - y_pos

        angleInRad = self.angle_of_vectors((u1,u2), (v1,v2))
        

        # angle = self.angle_of_vectors()
        #self.client.moveByRollPitchYawrateThrottleAsync(0, 0, 0, 0.6, 0.3, vehicle_name=vehicle_name).join()
        self.client.rotateToYawAsync(angleInRad, timeout_sec=0.1, margin=0.1, vehicle_name=vehicle_name)
        return angleInRad
        
        
    def turnTowardsGoalSynchronous(self, vehicle_name, lidar_names):
        print('turning towards goal')
        self.client.hoverAsync().join()
        dist = math.sqrt((self.destination[0]**2)+(self.destination[1]**2))
        #theta = math.acos()
        rotate = True
        isTurning = True

        # while rotate == True:
        #airsim.types.EnvironmentState.position
        # print(self.client.getMultirotorState().orientation)
        
        length = 5
        w_val, x_val, y_val, z_val = (self.client.getMultirotorState().kinematics_estimated.orientation)
        x_pos, y_pos, z_pos = (self.client.getMultirotorState().kinematics_estimated.position)
        roll_x, pitch_y, yaw_z = self.euler_from_quaternion(x_val, y_val, z_val, w_val) #yaw_z is the global radian angle of the drone
        points = [x_pos + length * math.cos(yaw_z), y_pos + length * math.sin(yaw_z)]
        # gps = self.client.getLidarData(lidar_name=lidar_names[0],vehicle_name=vehicle_name).pose.position
        # print(gps)

        u1 = self.destination[0] - x_pos
        u2 = self.destination[1] - y_pos

        v1 = points[0] - x_pos
        v2 = points[1] - y_pos

        angleInRad = self.angle_of_vectors((u1,u2), (v1,v2))
        

        # angle = self.angle_of_vectors()
        #self.client.moveByRollPitchYawrateThrottleAsync(0, 0, 0, 0.6, 0.3, vehicle_name=vehicle_name).join()
        # print('started turning towards goal')
        self.client.rotateToYawAsync(angleInRad, timeout_sec=10, margin=0.1, vehicle_name=vehicle_name).join()
        # print('finished turning towards goal')
        return angleInRad

    def euler_from_quaternion(self,x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
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
            
    def angle_of_vectors(self, vector1, vector2):
        [a, b] = vector1
        [c, d] = vector2

        dotProduct = a*c + b*d
            # for three dimensional simply add dotProduct = a*c + b*d  + e*f 
        modOfVector1 = math.sqrt( a*a + b*b)*math.sqrt(c*c + d*d) 
            # for three dimensional simply add modOfVector = math.sqrt( a*a + b*b + e*e)*math.sqrt(c*c + d*d +f*f) 
        angle = dotProduct/modOfVector1
        #  print("Cosθ =",angle)
        #  angleInDegree = math.degrees(math.acos(angle))
        angleInRad = math.acos(angle)
        #  print("θ =",angleInDegree,"°")

        return angleInRad

    def vision(self, overall_point_list):
      
        top_row = overall_point_list[1] # or 0?
        midpoint_top_level_ind = int(len(overall_point_list[1]) / 2)
        x_val = overall_point_list[1][midpoint_top_level_ind][0]
        y_val = overall_point_list[1][midpoint_top_level_ind][1]
        z_val = overall_point_list[1][midpoint_top_level_ind][2]

        return x_val, y_val, z_val

    def checkForCollision(self, overall_point_list):
        #stop = False
        
        x_val, y_val, z_val = self.vision(overall_point_list)
        # print(f"x val: {x_val}" )
        # print(f"y val: {y_val}" )
        # print(f"z val: {z_val}" )
        if( x_val <= self.collisiondist):
            # self.stopDrone()
            self.collision = True
            return True
        return False

    def goForward(self):
        print('Thrust Forward')
        self.client.moveByVelocityBodyFrameAsync(3, 0, 0,0.1)

    def stop(self):

        airsim.wait_key('Press any key to reset to original state')

        self.client.armDisarm(False)
        self.client.reset()

        self.client.enableApiControl(False)
        print("Done!\n")

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
                    print('Found new row')
                    i = row_index

        # print ("chose row" + str(i))
        return i


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

    def dataFixer(self, threshold, chosen_row):

        #Todo: place 'Gap' at the end / corner
        
        
        last_point = np.array([chosen_row[0][0], chosen_row[0][1]])  #list of x and y value of first point
        for ind, point_val in enumerate(chosen_row):
            # [point_x, point_y] = np.array([point_val[0], point_val[1]])
            temp = last_point - np.array([point_val[0], point_val[1]])
            dist = np.sqrt(temp[0]**2 + temp[1]**2)

            if dist > threshold:
                print('inserted')
                chosen_row.insert(ind, 'G')
            
            last_point = np.array([point_val[0], point_val[1]])

        return chosen_row
                
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
        
    def calculate_vector(self, fixed_row):
        #find left-most chunk of points between gaps

        first_point = fixed_row[0]
        last_point = fixed_row[0]
        for ind, point in enumerate(fixed_row):
            #this will break if a G is at the start of the row
            if point == 'G':
                last_point = fixed_row[ind - 1]
                break
            last_point = fixed_row[ind]
        
        vector = ((last_point[0] - first_point[0]), (last_point[1] - first_point[1]))
        return vector

    def calculate_sum_vector(self, fixed_row):
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


        
    

    def execute(self,vehicle_name,lidar_names):
        print("arming the drone...")
        
        self.client.armDisarm(True)

        self.takeoff()  
        
        airsim.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
        # self.client.moveToPositionAsync(20, 0, 0, 5).join()
        self.client.moveToPositionAsync(0, 0, -1, 5).join()
        self.client.hoverAsync().join()

        print('Scanning Has Started\n')
        print('Use Keyboard Interrupt \'CTRL + C\' to Stop Scanning\n')
        # [Go Forward, Turn Right, Turn Left]

        
        Armed = True

        #have a theta offset that will grow as the drone rotates away from the goal. keep rotating and growing offset until the drone sees all zeros.
        #move forward, and turn slowly towards the goal as the drone is moving


        #turn to goal when 

        

        try:
            
            overall_point_list = self.scan(vehicle_name,lidar_names)
            
            #booltuple = self.booleanArray_parser(10, overall_point_list, self.chooseRow(overall_point_list))
            chosenRowIndex = self.chooseRow(overall_point_list)
            booleanArray, blah, blas, bla = self.booleanArray_parser(10, overall_point_list, chosenRowIndex)
            print('Lidar row len: ' + str(len(overall_point_list[chosenRowIndex])))
            print('booleanArray row len: ' + str(len(booleanArray)))
            print ("booleanArray: ")
            print (booleanArray)
            print ("chosen Row: ")
            print(chosenRowIndex)
            print (overall_point_list[chosenRowIndex])
            self.wall_drawer(booleanArray, overall_point_list[chosenRowIndex])
            fixedchosenRow = self.dataFixer(1, overall_point_list[chosenRowIndex])
            # print('FIXED: ' + str(fixedchosenRow))
            print('Sum vector: ' + str(self.calculate_sum_vector(fixedchosenRow)))
            
            
            
            #while Armed == True:
            #     #move to goal
            #     overall_point_list = self.scan(vehicle_name,lidar_names)
            #     angleToGoal = self.turnTowardsGoal(vehicle_name, lidar_names) # completely faces goal
            #     if (self.checkForCollision(overall_point_list) == False):
            #         self.goForward() #go forward until collision
            #     # if collision event
            #     #self.turnTowardsGoalSynchronous(vehicle_name, lidar_names)
            #     thetaOffset = 0
            #     while(self.checkForCollision(self.scan(vehicle_name, lidar_names)) == True):
            #         self.client.rotateToYawAsync(angleToGoal + thetaOffset, timeout_sec=0.1, margin=0.1, vehicle_name=vehicle_name)
            #         thetaOffset = thetaOffset + 0.2
                

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
    lidarTest = LidarTest()
    try:
        lidarTest.execute(vehicle_name,lidar_names)
    except KeyboardInterrupt:
        lidarTest.stop()
          
    finally:
        # lidarTest.display_lidar()
        lidarTest.stop()

