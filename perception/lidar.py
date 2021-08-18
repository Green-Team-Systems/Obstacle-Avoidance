import setup_path
import airsim

import sys
import math
import time
import argparse
import pprint
import numpy


# TODO:
# Create different responses based on LiDar data returned: yaw for trees, vertical for walls, sloped for hills/ terrain
# Clean up everything in the loop

class Lidar():

    def __init__(self):

        # connect to the AirSim simulator
        # TODO Remove these functions and setup drone
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

    def execute(self):

        # # Allows for user input from terminal for quicker testing
        # xCord = input("Enter the X-Coordinate of destination:")
        # yCord = input("Enter the Y-Coordinate of destination:")
        # zCord = input("Enter the Z-Coordinate of destination:")
        # droneVelocity = input("Enter the speed at which to travel to the destination:")

        # Initialization of variables for location and speed
        xCord = -400
        yCord = 0
        zCord = -2
        droneVelocity = 8
        count = 0

        
        obstacleDetected = False
        print("arming the drone...")
        self.client.armDisarm(True)

        # Takeoff of Drone
        state = self.client.getMultirotorState()
        s = pprint.pformat(state)
        airsim.wait_key('Press any key to takeoff')
        self.client.takeoffAsync().join()
        state = self.client.getMultirotorState()

        # Waitkey to begin Lidar drone testing.
        airsim.wait_key('Press any key to get Lidar readings')

        # Move the drone to the coordinates initialized above
        # self.client.moveByVelocityAsync(xCord, yCord, zCord, droneVelocity)
        self.client.moveToPositionAsync(xCord, yCord, zCord, droneVelocity)

        # Loop for Lidar testing
        for i in range(1, 400):
            lidarData = self.client.getLidarData();
            if (len(lidarData.point_cloud) < 3):
                print("\tNo points received from Lidar data")
            else:
                points = self.parse_lidarData(lidarData)
                x_points, y_points, z_points = numpy.array(points[:, 0]), numpy.array(points[:, 1]), numpy.array(
                    points[:, 2])
                zeros_index = numpy.argwhere(x_points >= -0.1)
                max_val = numpy.max(points)
                pos_val = abs(lidarData.pose.position.x_val)
                print(max_val, pos_val)
                distance_away = round(max_val - abs(lidarData.pose.position.x_val), 3)
                print(distance_away)
                print(numpy.amax(points))
                print("\tReading %d: time_stamp: %d number_of_points: %d" % (i, lidarData.time_stamp, len(points)))
                # print("\t\tlidar position: %s" % (pprint.pformat(lidarData.pose.position)))
                print("\t\tlidar orientation: %s" % (pprint.pformat(lidarData.pose.orientation)))
                for distance in z_points:
                    # print(distance)
                    # print(points)
                    if (distance < 3):
                        self.client.hoverAsync()
                        print('Obstacle Detected!')
                        obstacleDetected = True
                if (obstacleDetected == True):
                    print('Moving Up')
                    print(z_points[0])
                    print(len(z_points))
                    print(z_points[len(z_points) - 1])
                    print('End Points')
                    terrainSquared = abs(z_points[0] - (z_points[len(z_points) - 1]))
                    terrain = math.sqrt(terrainSquared)
                    self.client.moveToPositionAsync(lidarData.pose.position.x_val - terrain,
                                                    lidarData.pose.position.y_val,
                                                    lidarData.pose.position.z_val - terrainSquared, droneVelocity)
                    time.sleep(1)
                    print(lidarData.pose.position.x_val)
                    self.client.moveToPositionAsync(xCord, yCord, zCord, droneVelocity)
                    obstacleDetected = False

                    # for distance in z_points:
                    #    if(distance < 15):
                    #        print('Large distance.')
                    #        self.client.moveToPositionAsync(lidarData.pose.position.x_val,lidarData.pose.position.y_val,lidarData.pose.position.z_val + 5,droneVelocity).join()
                    #        count = count + 1
                    # self.client.moveToPositionAsync(xCord,yCord,zCord,droneVelocity)
            time.sleep(.5)

    def parse_lidarData(self, data):

        # reshape array of floats to array of [X,Y,Z]
        points = numpy.array(data.point_cloud, dtype=numpy.dtype('f4'))
        points = numpy.reshape(points, (int(points.shape[0] / 3), 3))

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

    # # Attempting to use command line arguments for location data
    # xCord = sys.argv[0]
    # print(xCord)
    # yCord = sys.argv[1]
    # zCord = sys.argv[2]
    # droneVelocity = sys.argv[3]

    arg_parser = argparse.ArgumentParser("Lidar.py makes drone fly and gets Lidar data")

    arg_parser.add_argument('-save-to-disk', type=bool, help="save Lidar data to disk", default=False)

    args = arg_parser.parse_args(args)
    lidar = Lidar()
    try:
        lidar.execute()
    except Exception as error:
        print(error)
    finally:
        lidar.stop()