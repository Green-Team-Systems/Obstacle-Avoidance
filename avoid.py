from numpy.core import arrayprint
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
        x_points = []
        y_points = []
        z_points = []
        d = {}
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
        self.client.moveToPositionAsync(0, 5, 0, 5).join()

        self.client.hoverAsync().join()

        time.sleep(20)
        
        f = open('airsimdata.txt', 'w')
        
        # if (len(lidar_data.point_cloud) < 3):
        #             print("\tNo points received from Lidar data")
        #         # Intended to Unrotate after a rotation was completed to avoid collision.
        #         # Currently it rotates to a static Yaw value and will need to be adjusted for relative values.
        # else:
        #         # Divides the lidarData into 3 seperate variables for x, y and z
        #         y_points_last = 0
        #         length = len(lidar_data.point_cloud)
        #         counter = 1
        #         overall_point_list = list()
        #         next_row = list()
        #         for i in range(0, len(lidar_data.point_cloud), 3):
        #             xyz = lidar_data.point_cloud[i:i+3]
        #             if (xyz[1] != math.fabs(xyz[1]) and y_points_last == math.fabs(y_points_last)):
        #                 overall_point_list.append(next_row)
        #                 next_row = list()
        #             next_row.append(xyz)
        #             f.write("%f %f %f\n" % (xyz[0],xyz[1],-xyz[2]))
        #             y_points_last = xyz[1]
        
        # midpoint_top_level = int(len(overall_point_list[1]) / 2)
        # x2_distance = overall_point_list[1][midpoint_top_level][0]
        # z2_distance = -overall_point_list[1][midpoint_top_level][2]

        # bottom_level_point = len(overall_point_list) - 1
        # midpoint_bottom_level = int(len(overall_point_list[bottom_level_point]) / 2)
        # x1_distance = overall_point_list[bottom_level_point][midpoint_bottom_level][0]
        # z1_distance = -overall_point_list[bottom_level_point][midpoint_bottom_level][2]

        # x_distance = x2_distance - x1_distance
        # z_distance = z2_distance - z1_distance

        # LidarHypo = math.sqrt(math.pow(x_distance, 2) + math.pow(z_distance, 2))
        # distanceHypo = math.sqrt(math.pow(ballsData.distance, 2) + math.pow(cockData.distance, 2))

        # print(f'Lidar hypo {LidarHypo}')
        # print(f'Distance hypo {distanceHypo}')


        # Loop for Lidar testing
        # The variable value of 400 was used to ensure a long enough time to thoroughly test
        while 1 == 1:

            # see getLidarData below.
            lidar_data = self.client.getLidarData()
            
            # Depending on the range of the Lidar sensor (in the settings.json) no points will be recieved if the points distance exceeds the range.
            # Depending on the range of the Lidar sensor (in the settings.json) no points will be recieved if the points distance exceeds the range.
            if (len(lidar_data.point_cloud) < 3):
                    print("\tNo points received from Lidar data")
                # Intended to Unrotate after a rotation was completed to avoid collision.
                # Currently it rotates to a static Yaw value and will need to be adjusted for relative values.
            else:
                    # Divides the lidarData into 3 seperate variables for x, y and z
                    y_points_last = 0
                    length = len(lidar_data.point_cloud)
                    overall_point_list = list()
                    next_row = list()
                    for i in range(0, len(lidar_data.point_cloud), 3):
                        xyz = lidar_data.point_cloud[i:i+3]
                        if (xyz[1] != math.fabs(xyz[1]) and y_points_last == math.fabs(y_points_last)):
                            overall_point_list.append(next_row)
                            next_row = list()
                        next_row.append(xyz)
                        f.write("%f %f %f\n" % (xyz[0],xyz[1],-xyz[2]))
                        y_points_last = xyz[1]
            
            midpoint_top_level = int(len(overall_point_list[1]) / 2)
            x2_distance = overall_point_list[1][midpoint_top_level][0]
            z2_distance = -overall_point_list[1][midpoint_top_level][2]

            bottom_level_point = len(overall_point_list) - 1
            midpoint_bottom_level = int(len(overall_point_list[bottom_level_point]) / 2)
            x1_distance = overall_point_list[bottom_level_point][midpoint_bottom_level][0]
            z1_distance = -overall_point_list[bottom_level_point][midpoint_bottom_level][2]

            x_distance = x2_distance - x1_distance
            z_distance = z2_distance - z1_distance

            hypo = math.sqrt(math.pow(x_distance, 2) + math.pow(z_distance, 2)) 
            zVelocity = (z_distance / hypo)
            xVelocity = (x_distance / hypo)
            zVelocity = zVelocity * droneVelocity
            xVelocity = xVelocity * droneVelocity
        


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