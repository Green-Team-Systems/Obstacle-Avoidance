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
        droneVelocity = 2 # meters / second
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
        for i in range(1,400):

            # see getLidarData below.
            lidarData = self.client.getLidarData();

            # Depending on the range of the Lidar sensor (in the settings.json) no points will be recieved if the points distance exceeds the range.
            if (len(lidarData.point_cloud) < 3):
                print("\tNo points received from Lidar data")

                # Intended to Unrotate after a rotation was completed to avoid collision.
                # Currently it rotates to a static Yaw value and will need to be adjusted for relative values.
                if(rotation != 0.0):
                    self.client.rotateToYawAsync(180,10,1).join()
            else:
                # Divides the lidarData into 3 seperate variables for x, y and z
                points = self.parse_lidarData(lidarData)
                x_points, y_points, z_points = numpy.array(points[:, 0]),numpy.array(points[:, 1]), numpy.array(points[:,2])

                # Unused calculations that I kept in the code.
                zeros_index = numpy.argwhere(x_points >= -0.1)
                max_val = numpy.max(points)
                pos_val = abs(lidarData.pose.position.x_val)
                print(max_val, pos_val)
                distance_away = round(max_val - abs(lidarData.pose.position.x_val), 3)
                print(distance_away)
                print(numpy.amax(points))

                # Returns timing data and other useful information for testing of Lidar
                print("\tReading %d: time_stamp: %d number_of_points: %d" % (i, lidarData.time_stamp, len(points)))
                # print("\t\tlidar position: %s" % (pprint.pformat(lidarData.pose.position)))
                print("\t\tlidar orientation: %s" % (pprint.pformat(lidarData.pose.orientation)))

                # The z- points were used since these are always positive and correspond to the distance of the
                # Lidar sensor from the returned point. This for loop is essentially looping through all the 
                # returned points and checking if they are less than 'collisionImminent'.
                # If they are less than this value, then 'obstacleDetected' is set to true.
                for distance in z_points:
                    #print(distance)
                    #print(points)
                    if (distance < collisionImminent):
                        self.client.hoverAsync()
                        print('Obstacle Detected!')
                        obstacleDetected = True

                # Only entered if an Obstacle is detected by being less than 'collisionImminent'
                if(obstacleDetected == True):
                    print('Avoiding')

                    # # These values are printed to test returned points for implementing parralel terrain traversal.
                    #print(z_points[0])
                    #print(len(z_points))
                    #print(z_points[len(z_points) - 1])
                    #print('End Points')

                    # Calculations used to ascend a mountain by using the slope of the terrain.
                    terrainSquared = abs(z_points[0] - (z_points[len(z_points) - 1]))
                    terrain = math.sqrt(terrainSquared)

                    # # This block was an attempt to yaw based on the first and last returned points for the lidar data.
                    # # It doesn't currently work.
                    #if(z_points[0] >= 10):
                    #    self.client.rotateToYawAsync(5, 10, 1)
                    #elif(z_points[len(z_points) - 1] >= 10):
                    #    self.client.rotateToYawAsync(-5, 10, 1)

                    # # This block completes the yaw-based collision avoidance. Currently -90 is a hard set value
                    # # This will need to be adjusted to be relative.
                    # # .join() is used to make sure the command is completed before continuing. 
                    #self.client.rotateByYawRateAsync(5, 1, 1)
                    # self.client.rotateToYawAsync(-90,10,1).join()
                    # rotation = rotation - 1
                    # print('\n\n\n\n Rotating')

                    # # These block is to activate the collision avoidance based on slope calculation.
                    # # .join() is not used to keep functions from stepping on each other.
                    self.client.moveToPositionAsync(
                        lidarData.pose.position.x_val - terrain,
                        lidarData.pose.position.y_val,
                        (lidarData.pose.position.z_val
                        - terrainSquared,droneVelocity)
                    )
                    #time.sleep(1)
                    #print(lidarData.pose.position.x_val)

                    # Calculations to nudge the drone in the correct direction after yawing to avoid a collision.
                    # yaw = rotation
                    # vx = math.cos(yaw) * droneVelocity
                    # vy = math.sin(yaw) * droneVelocity
                    # self.client.moveByVelocityZAsync(vx, vy, 0, 1, 1)



                    # # This block is to reset after a loop and give time to complete.
                    # self.client.moveByVelocityAsync(xCordNew,yCordNew,zCord,droneVelocity)
                    time.sleep(1)
                    obstacleDetected = False

                    # # This block was the original terrain traversal. It ascends the drone whenever an object is detected.
                    # # I kept it because it will be needed for vertical terrain (walls) since the calculated 'terrain' will cause
                    # # the drone to be unmoving.
                    #for distance in z_points:
                    #    if(distance < 15):
                    #        print('Large distance.')
                    #        self.client.moveToPositionAsync(lidarData.pose.position.x_val,lidarData.pose.position.y_val,lidarData.pose.position.z_val + 5,droneVelocity).join()
                    #        count = count + 1
                    #self.client.moveToPositionAsync(xCord,yCord,zCord,droneVelocity)

                    # # With this block, the yawing blocks, and the Terrain blocks, there are 3 types of obstacle avoidance depending on what type of avoidance is needed.



            # # This block is another attempt to undo rotation, it would need to be adjusted to be relative.
            #if((obstacleDetected == False) & (rotation != 0)):
            #    self.client.rotateToYawAsync(5,1,1)
            #    rotation = rotation + 1

            # # Resend the original coordinates so the drone doesn't stay too long on the actions of one loop
            # # If further adjustment is needed, this will occur on the next pass.
            self.client.moveToPositionAsync(xCord, yCord, zCord, droneVelocity, 1)
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

    # # Attempting to use command line arguments for location data
    # xCord = sys.argv[0]
    # yCord = sys.argv[1]
    # zCord = sys.argv[2]
    # droneVelocity = sys.argv[3]

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