import setup_path 
import airsim

import sys
import math
import time
import argparse
import pprint
import numpy

class LidarTest:

    def __init__(self):

        # connect to the AirSim simulator
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

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


    def takeoff(self):

        # Takeoff of Drone
        state = self.client.getMultirotorState()
        s = pprint.pformat(state)
        print(s)
        airsim.wait_key('Press any key to takeoff')
        self.client.takeoffAsync().join()
        state = self.client.getMultirotorState()   


    def get_Lidar(self, collisionImminent, droneVelocity, obstacleDetected):
        """
        This function gets lidar data, decides if collision is imminent, and sets new trajectory
        """


        # Lidar testing
        for i in range(1,400):
            lidarData = self.client.getLidarData()

            # Depending on the range of the Lidar sensor (in the settings.json) no points will be recieved if the points distance exceeds the range.
            if (len(lidarData.point_cloud) < 3):
                print("\tNo points received from Lidar data")


            else:
                # Divides the lidarData into 3 seperate variables for x, y and z
                points = self.parse_lidarData(lidarData)
                x_points, y_points, z_points = numpy.array(points[:, 0]),numpy.array(points[:, 1]), numpy.array(points[:,2])

                # Returns timing data and other useful information for testing of Lidar
                # print("\tReading %d: time_stamp: %d number_of_points: %d" % (i, lidarData.time_stamp, len(points)))
                # print("\t\tlidar orientation: %s" % (pprint.pformat(lidarData.pose.orientation)))


                # looping through returned points and checking if they are less than 'collisionImminent'.
                # If they are less than this value, then 'obstacleDetected' is set to true.
                for distance in z_points:

                    if (distance < collisionImminent):
                        self.client.hoverAsync()
                        print('Obstacle Detected!')
                        obstacleDetected = True

                # Only entered if an Obstacle is detected by being less than 'collisionImminent'
                if(obstacleDetected == True):
                    print('Avoiding')

                    # Calculations used to ascend a mountain by using the slope of the terrain.
                    terrainSquared = abs(z_points[0] - (z_points[len(z_points) - 1]))
                    terrain = math.sqrt(terrainSquared)

                    # # These block is to activate the collision avoidance based on slope calculation.
                    # # .join() is not used to keep functions from stepping on each other.
                    self.client.moveToPositionAsync(
                        lidarData.pose.position.x_val - terrain,
                        lidarData.pose.position.y_val,
                        (lidarData.pose.position.z_val - terrainSquared),
                        droneVelocity
                        )

                    time.sleep(1)
                    obstacleDetected = False


    def execute(self):
        # Initialization of variables and drone
        xCord = 400 # meters
        yCord = 0 # meters
        zCord = 0 # meters
        droneVelocity = 2 # meters / second
        collisionImminent = 3
        rotation = 0
        xCordNew = (xCord * math.cos(rotation) + yCord * math.sin(rotation)) / (xCord + yCord)
        yCordNew = (-xCord * math.sin(rotation) + yCord * math.cos(rotation)) / (xCord + yCord)
        obstacleDetected = False
        print("arming the drone...")
        self.client.armDisarm(True)

        # drone takeoff
        LidarTest.takeoff(self)

        # Waitkey to begin Lidar drone testing.
        airsim.wait_key('Press any key to get Lidar readings')

        self.client.moveToPositionAsync(xCord, yCord, zCord, droneVelocity)

        # Get lidar data and calculate for obstacle avoidance 
        LidarTest.get_Lidar(self, collisionImminent, droneVelocity, obstacleDetected)

        #self.client.moveToPositionAsync(xCord, yCord, zCord, droneVelocity, 1)
        time.sleep(1)




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