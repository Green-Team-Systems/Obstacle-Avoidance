
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

    def execute(self):

        #xCord = input("Enter the X-Coordinate of destination:")

        #yCord = input("Enter the Y-Coordinate of destination:")

        #zCord = input("Enter the Z-Coordinate of destination:")

        #droneVelocity = input("Enter the speed at which to travel to the destination:")

        xCord = 400
        yCord = 0
        zCord = -5
        droneVelocity = 8
        count = 0
        print("arming the drone...")
        self.client.armDisarm(True)

        state = self.client.getMultirotorState()
        s = pprint.pformat(state)
        #print("state: %s" % s)

        airsim.wait_key('Press any key to takeoff')
        self.client.takeoffAsync().join()

        state = self.client.getMultirotorState()    
        #print("state: %s" % pprint.pformat(state))

        # airsim.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
        # self.client.moveToPositionAsync(0, 30, -1, 5)

        #  self.client.hoverAsync().join()

        
        airsim.wait_key('Press any key to get Lidar readings')
        #self.client.moveToPositionAsync(400,0,0,10)
        obstacleDetected = False

        # self.client.moveByVelocityAsync(2, 2, 0, droneVelocity)
        self.client.moveToPositionAsync(xCord, yCord, zCord, droneVelocity)
        for i in range(1,400):
            lidarData = self.client.getLidarData();
            if (len(lidarData.point_cloud) < 3):
                print("\tNo points received from Lidar data")
            else:
                points = self.parse_lidarData(lidarData)
                x_points, y_points, z_points = numpy.array(points[:, 0]),numpy.array(points[:, 1]), numpy.array(points[:,2])
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
                    #print(distance)
                    #print(points)
                    if (distance < 3):
                        self.client.hoverAsync()
                        print('Obstacle Detected!')
                        obstacleDetected = True
                        #break
                if(obstacleDetected == True):
                    #break
                    #zCord = zCord - 5
                    #self.client.moveToPositionAsync(0,0,50, droneVelocity)
                    
                    print('Moving Up')
                    self.client.moveToPositionAsync(lidarData.pose.position.x_val,lidarData.pose.position.y_val,lidarData.pose.position.z_val - 5,droneVelocity)
                    #Seconds to wait
                    time.sleep(1)
                    self.client.moveToPositionAsync(xCord,yCord,zCord,droneVelocity)
                    obstacleDetected = False

                    #for distance in z_points:
                    #    if(distance < 15):
                    #        print('Large distance.')
                    #        self.client.moveToPositionAsync(lidarData.pose.position.x_val,lidarData.pose.position.y_val,lidarData.pose.position.z_val + 5,droneVelocity).join()
                    #        count = count + 1
                    #self.client.moveToPositionAsync(xCord,yCord,zCord,droneVelocity)
            time.sleep(1)

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

    #Attempting to use command line arguments for location data
    #xCord = sys.argv[0]
    #print(xCord)
    #yCord = sys.argv[1]
    #zCord = sys.argv[2]
    #droneVelocity = sys.argv[3]

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