import setup_path 
import airsim

import sys
import math
import time
import argparse
import pprint
import numpy

# TODO: 
# Create different responses based on LiDar data returned: yaw for trees,
# vertical for walls, sloped for hills/ terrain
# Clean up everything in the loop


class LidarTest:

    def __init__(self):

        # connect to the AirSim simulator
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

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
        count = 0
        rotation = 0.0 # TODO Fifgure this out
        collisionImminent = 3
        xCordNew = (xCord * math.cos(rotation) + yCord * math.sin(rotation)) / (xCord + yCord)
        yCordNew = (-xCord * math.sin(rotation) + yCord * math.cos(rotation)) / (xCord + yCord)
        obstacleDetected = False
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
        self.client.moveToPositionAsync(xCord, yCord, zCord, droneVelocity)

        # Loop for Lidar testing
        # The variable value of 400 was used to ensure a long enough time to thoroughly test
        while 1 == 1:

            # see getLidarData below.
            lidarData = self.client.getLidarData()
            ballsData = self.client.getDistanceSensorData(distance_sensor_name = "Balls")
            cockData = self.client.getDistanceSensorData(distance_sensor_name = "Cock")

            # prints the data collected from the distance sensors
            print(f'Z-Distance sensor data: {ballsData.distance}')
            print(f'X-Distance sensor data: {cockData.distance}')

            #slope = ballsData.distance / cockData.distance
            hypo = math.sqrt(math.pow(ballsData.distance, 2) + math.pow(cockData.distance, 2))
            zVelocity = ballsData.distance / hypo 
            xVelocity = cockData.distance / hypo 
            if cockData.distance < (0.5 * droneVelocity):
                zVelocity = droneVelocity
                xVelocity = 0
            elif ballsData.distance < 10 or cockData.distance < (0.75 * droneVelocity):
                zVelocity = zVelocity * droneVelocity
                xVelocity = xVelocity * droneVelocity
            elif ballsData.distance > 20:
                zVelocity = (10 - ballsData.distance)
                xVelocity = 0
            else:
                zVelocity = 0 
                xVelocity = droneVelocity


            print(f'Z speed: {zVelocity}')
            print(f'X speed: {xVelocity}')
            
            self.client.moveByVelocityAsync(xVelocity, 0, -zVelocity, 0.01)
            time.sleep(0.01)

    def parse_lidarData(self, data):

        # reshape array of floats to array of [X,Y,Z]
        points = numpy.array(data.point_cloud, dtype=numpy.dtype('f4'))
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