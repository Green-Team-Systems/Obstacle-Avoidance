from numpy.core import arrayprint
import setup_path
import airsim

import sys 
import math
import time
import argparse
import pprint
import numpy as np

from datetime import datetime

class LidarTest:

    def __init__(self):
        """
        Initializes the drone and all necessary variables.
        """

        # connect to the AirSim simulator
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

        self.droneVelocity = 5 # set max drone velocity ---> m/s

        # set up variables for PID
        self.errors = {
            "X": 0.0,
            "Y": 0.0,
            "Z": 0.0
        }
        self.integral_error = {
            "X": 0.0,
            "Y": 0.0,
            "Z": 0.0
        }
        self.previous_velocities = {
            "VX": 0.0,
            "VY": 0.0,
            "VZ": 0.0,
        }
        self.current_time = datetime.utcnow()
        self.previous_time = datetime.utcnow()

    def calculate_velocities(self, x_error, y_error):
        """
        Calculates x and y velocities based off of the x and y error. 
        Stolen from Avi's code.

        :param x_error: how far the drone needs to travel in the 
            x-direction ---> meters

        :param y-error: how far the drone needs to travel in the
            y-direction ---> meters

        :return x_vel, y_vel: returns a velocity in the x-direction and
            y-direction based off of the given error ---> m/s
        """

        z_error = 0 # temp variable for integration

        kP = 0.4
        kI = 0.00
        kD = 0.05

        now = datetime.utcnow()
        dt = (now - self.previous_time).total_seconds()
        self.previous_time = now

        x_derivative = (x_error - self.errors["X"]) / dt
        y_derivative = (y_error - self.errors["Y"]) / dt
        z_derivative = (z_error - self.errors["Z"]) / dt

        x_integral = (self.integral_error["X"] + x_error) * dt
        y_integral = (self.integral_error["Y"] + y_error) * dt
        z_integral = (self.integral_error["Z"] + z_error) * dt

        x_vel = (((x_error) * (kP))
                 + ((x_derivative) * (kD))
                 + ((x_integral) * (kI)))
        y_vel = (((y_error) * (kP))
                 + ((y_derivative) * (kD))
                 + ((y_integral) * (kI)))
        z_vel = (((z_error) * (kP))
                 + ((z_derivative) * (kD))
                 + ((z_integral) * (kI)))

        x_vel = self.apply_velocity_constraints(x_vel)
        y_vel = self.apply_velocity_constraints(y_vel)
        z_vel = self.apply_velocity_constraints(z_vel, True)

        self.errors = {
            "X": x_error,
            "Y": y_error,
            "Z": z_error
        }

        return x_vel, y_vel

    def apply_velocity_constraints(self, speed, z_val=False):
        """
        Corrects the output velocities so they are not outside of the 
        given constraints. Also stolen from Avi's code.

        :param speed: current calculated velocity ---> m/s

        :param x_val: boolean value for whether or not the given speed
            is on the x-axis. defaults to false if not specified

        :return speed: returns the corrected velocity ---> m/s 
        """
        if not z_val and speed > self.droneVelocity:
            speed = self.droneVelocity
        elif z_val and speed > 5:
            speed = 5
        return speed

    def detectCorners(self, lidarData):
        """
        Detects corners/edges of objects, and classifies the type of point 
        it is. Based on this classification, the system will then determine
        an appropriate position to travel to or velocities to travel at.
        If it finds a position to travel to, the function will call a 
        seperate PID function in order to calculate velocities for 
        travelling to that position

        :param lidarData: this is the full array of points output by
            the LiDAR. ---> meters

        :return xVelocity, yVelocity: returns a velocity in the
            x-direction and a velocit in the y-direction. ---> m/s

        """

        # Depending on the range of the Lidar sensor (in the settings.json) 
        # no points will be recieved if the points distance exceeds the range.
        if (len(lidarData) < 3):
                print("\tNo points received from Lidar data")
                xVelocity = self.droneVelocity
                yVelocity = 0
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

                # find index of the middle row
                top_row = overall_point_list[1]
                mid_point_top_level_ind = int(len(top_row) / 2)
                z_level_row = top_row[mid_point_top_level_ind][2] # initial z val of top row mid
                index = 1

                for row_index, row in enumerate(overall_point_list):
                    if(len(row) > 0):
                        mid_point = int(len(row) / 2) # index of mid point
                        mid_point_z = row[mid_point][2] # z value of mid point
                        if(abs(mid_point_z) < abs(z_level_row)):
                            z_level_row = mid_point_z
                            index = row_index

                try:
                    mid_row = overall_point_list[index]
                except Exception:
                    mid_row = []
                
                target = np.array([-999, -999])
                newTarget = np.zeros(2)
                type = 'na'

                # find critical points
                for i in range(1, len(mid_row)):
                    yChange = mid_row[i][1] - mid_row[i-1][1]
                    xChange = mid_row[i][0] - mid_row[i-1][0]
                    zVal = ((mid_row[i][2] + mid_row[i-1][2]) / 2)
                    zBool = zVal > -3 and zVal < 3

                    if abs(yChange) > 2 and abs(xChange) < 10 and zBool:
                        points = np.array([mid_row[i-1][0], mid_row[i-1][1],
                            mid_row[i][0], mid_row[i][1]])
                        newTarget[0] = ((points[2] + points[0]) / 2) - 1
                        newTarget[1] = (points[3] + points[1]) / 2
                        width = abs(points[2] - points[0])
                        if abs(newTarget[1]) < abs(target[1]) and abs(target[1]) > 0.5 and width < 2:
                            target[0] = newTarget[0]
                            target[1] = newTarget[1]
                            type = 'yd'

                    elif xChange > 5 and mid_row[i-1][0] > 5 and zBool:
                        newTarget[0] = mid_row[i-1][0]
                        newTarget[1] = mid_row[i-1][1] + 2
                        if abs(newTarget[1]) < abs(target[1]) and abs(target[1]) > 1 and type != 'yd':
                            target[0] = newTarget[0] 
                            target[1] = newTarget[1]
                            type = 'xj'

                    elif xChange < -5 and mid_row[i][0] > 5 and zBool:
                        newTarget[0] = mid_row[i-1][0]
                        newTarget[1] = mid_row[i-1][1] - 2 
                        if abs(newTarget[1]) < abs(target[1]) and abs(target[1]) > 1 and type != 'yd':
                            target[0] = newTarget[0]
                            target[1] = newTarget[1]
                            type = 'xd'
 
                if type == 'yd' or type == 'xd' or type == 'xj':
                    xVelocity, yVelocity = self.calculate_velocities(target[0], target[1])
                else:
                    xVelocity, yVelocity = self.droneVelocity, 0

                try:
                    # prints velocities and targets
                    print(f'X speed: {xVelocity}')
                    print(f'Y speed: {yVelocity}')
                    print(f'X target: {target[0]}')
                    print(f'Y target: {target[1]}')
                    print(f'Type: {type}')

                # runs if points_list is empty
                except Exception:
                    xVelocity = 5
                    yVelocity = 0
                    print('exception')

        return xVelocity, yVelocity

    def execute(self):
        """
        Execute all necessary functions for the drone to operate.
        """


        # Initialization of variables and drone
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
        # self.client.moveToPositionAsync(0, 5, 0, 5).join()

        self.client.hoverAsync().join()

        #time.sleep(10)

        # f = open('airsimdata.txt', 'w')

        while 1:

            # lidar_data and data both need to be placed in a while loop along with function defination to make sure it works :)
            lidar_data = self.client.getLidarData()
            data = lidar_data.point_cloud

            x_vel, y_vel = self.detectCorners(data) # takes the data as one parameter and 10 is drone speed user inputs

            self.client.moveByVelocityAsync(x_vel, y_vel, 0, math.inf)
            time.sleep(0.01)

    def parse_lidarData(self, data):
        """
        Why is this here it's never used lmao.
        """
        # reshape array of floats to array of [X,Y,Z]
        points = np.array(data.point_cloud, type=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0]/3), 3))

        return points

    def write_lidarData_to_disk(self, points):
        """
        Why is this here too lmao, seems kinda pointless
        """
        # TODO
        print("not yet implemented")

    def stop(self):
        """
        Stops the drone and resets
        """

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