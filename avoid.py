from numpy.core import arrayprint
import setup_path 
import airsim

import sys
import math
import time
import argparse
import pprint
import numpy as np

# TODO: 
# Create different responses based on LiDar data returned: yaw for trees,
# vertical for walls, sloped for hills/ terrain
# Clean up everything in the loop

class LidarTest:

    def __init__(self):

        # connect to the AirSim simulator
        self.client = airsim.MultirotorClient()
        self.starting_pos = self.client.getMultirotorState().kinematics_estimated.position
        self.client.confirmConnection()
        self.client.enableApiControl(True)
    
    
    def parse_liadar_date(self, lidarData):
        if (len(lidarData) < 3):
                print("\tNo points received from Lidar data")
            # Intended to Unrotate after a rotation was completed to avoid collision.
            # Currently it rotates to a static Yaw value and will need to be adjusted for relative values.
        else:
                # Divides the lidarData into 3 seperate variables for x, y and z
                y_points_last = 0
                length = len(lidarData)
                overall_point_list = list()
                next_row = list()
                for i in range(0, length, 3):
                    xyz = lidarData[i:i+3]
                    if (xyz[1] != math.fabs(xyz[1]) and y_points_last == math.fabs(y_points_last)):
                        overall_point_list.append(next_row)
                        next_row = list()
                    next_row.append(xyz)
                    y_points_last = xyz[1]
        return overall_point_list
    
    def moveForward(self):
        self.client.moveByVelocityAsync(6, 0, 0, math.inf) 
    
    def moveRight(self):
        self.client.moveByVelocityAsync(0, 2, 0, math.inf) 

    def slopeCalculation(self, lidarData, droneVelocity):
    # Depending on the range of the Lidar sensor (in the settings.json) no points will be recieved if the points distance exceeds the range.
        #state_data = self.client.getMultirotorState()
        overall_point_list = self.parse_liadar_date(lidarData)
        top_level = overall_point_list[1]
        x_points = []
        min = 1000

        for i in range(len(top_level)):
            x_point  = top_level[i][0]
            y_point = top_level[i][1]

            if (y_point > -1.0 and y_point < 1.0):
                x_points.append(x_point)
                if min > x_point:
                    min = x_point
        
        if min < 10:
            self.moveRight()
            return [0, 1, 0]
        else:
            self.moveForward()
            return [1, 0, 0]

    def execute(self):
        """
        This is where your function description.
        """
        # Allows for user input of location and velocity variables from terminal for quicker testing
        # xCord = input("Enter the X-Coordinate of destination:")
        # yCord = input("Enter the Y-Coordinate of destination:")
        # zCord = input("Enter the Z-Coordinate of destination:")
        # droneVelocity = input("Enter the speed at which to travel to the destination:")

        # Initialization of variables and drone
        xCord = 400 # meters
        yCord = 0 # meters
        zCord = 0 # meters
        droneVelocity = 10 # meters / second
        rotation = 0.0 # TODO Fifgure this out
        print("arming the drone...")
        self.client.armDisarm(True)

        # Takeoff of Drone
        state = self.client.getMultirotorState()
        s = pprint.pformat(state)
        print(s)
        airsim.wait_key('Press any key to takeoff')
        self.client.takeoffAsync().join()
        state = self.client.getMultirotorState()    
        
        # Waitkey to begin Lidar drone testing.
        airsim.wait_key('Press any key to get Lidar readings')

        # Move the drone to the coordinates initialized above
        # MovetoPosition will use Airsim Path Planning while moveByVelocity will not, i.e. moveByVelocity will be less jumpy.
        # self.client.moveByVelocityAsync(xCord, yCord, zCord, droneVelocity)
        #self.client.moveToPositionAsync(0, 5, 0, 5).join()

        self.client.hoverAsync().join()

        #time.sleep(10)
        
        # f = open('airsimdata.txt', 'w')

        while 1:

            # lidar_data and data both need to be placed in a while loop along with function defination to make sure it works :) 
            lidar_data = self.client.getLidarData()  
            data = lidar_data.point_cloud

            self.slopeCalculation(data, 10) #takes the data as one parameter and 10 is drone speed user inputs
            
            #self.client.moveByVelocityAsync(10, 0, 0, 0.01) 
            time.sleep(0.01)

    def parse_lidarData(self, data):

        # reshape array of floats to array of [X,Y,Z]
        points = numpy.array(data.point_cloud, type=numpy.dtype('f4'))
        points = numpy.reshape(points, (int(points.shape[0]/3), 3))
       
        return points

    def write_lidarData_to_disk(self, points):
        # TODO
        print("not yet implemented")

    def stop(self):

        airsim.wait_key('Press any key to reset to original state')

        self.client.armDisarm(False)
        self.client.reset()

        self.client.enableApiControl(False)
        print("Done!\n")

# main
if __name__ == "__main__":
    args = sys.argv
    args.pop(0)
    arg_parser = argparse.ArgumentParser("Lidar.py makes drone fly and gets Lidar data")

    arg_parser.add_argument('-save-to-disk', type=bool, help="save Lidar data to disk", default=False)
  
    args = arg_parser.parse_args(args)    
    lidarTest = LidarTest()
    try:
        lidarTest.execute()
    except Exception as error:
        print(error)
    finally:
        lidarTest.stop()