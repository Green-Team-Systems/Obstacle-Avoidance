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


# Makes the drone fly and get Lidar data
class LidarTest:
    
    destination = (100,0,0)
    collisiondist = 3
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
            # self.collision = True
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


    def booleanArray_parser(self, overall_point_list):
        booleanArray = list()
        midline = overall_point_list[1]
        # threshold = 5
        
        
        #trim midline to set 0s in boolean array where any pairs have x > 5
        #trim y-values to only look at middle 50 points

        # Array construction
        x_list = []
        for xval,yval,zval in midline:
            x_list.append(xval)

        midline_x_val = x_list[int(len(x_list)/2)]
        # print(midline_x_val)

        

        for x in x_list:
            if x < self.collisiondist + 10: # test narrower and shorter narrowview
                booleanArray.append(1)
            else:
                booleanArray.append(0)
        
        midpoint_booleanArray_ind = int(len(booleanArray)/2) # Mid index

        leftHalf_booleanArray = booleanArray[0:midpoint_booleanArray_ind]
        rightHalf_booleanArray = booleanArray[midpoint_booleanArray_ind:len(booleanArray)]

        # middle n points
        offset = 200
        narrow_view = booleanArray[midpoint_booleanArray_ind-offset:midpoint_booleanArray_ind+offset]
        narrow_view = np.array(narrow_view)

        return booleanArray, leftHalf_booleanArray, rightHalf_booleanArray, narrow_view


    def execute(self,vehicle_name,lidar_names):
        print("arming the drone...")
        
        self.client.armDisarm(True)

        self.takeoff()
        
        airsim.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
        # self.client.moveToPositionAsync(20, 0, 0, 5).join()

        self.client.hoverAsync().join()

        print('Scanning Has Started\n')
        print('Use Keyboard Interrupt \'CTRL + C\' to Stop Scanning\n')
        # [Go Forward, Turn Right, Turn Left]

        Armed = True

        #turn to goal when 
        # global collision
        # collision = False
        
        angleToGoal = self.turnTowardsGoal(vehicle_name, lidar_names) # completely faces goal


        while Armed == True:
            #move to goal
            overall_point_list = self.scan(vehicle_name,lidar_names)
            
            booleanArray,leftHalf_booleanArray,rightHalf_booleanArray,narrow_view = self.booleanArray_parser(overall_point_list)
            # if collision event
            

            # if((not np.any(narrow_view)) == True): # if narrow view is 0's cleared to go
                # self.goForward()
            booleanArray,leftHalf_booleanArray,rightHalf_booleanArray,narrow_view = self.booleanArray_parser(self.scan(vehicle_name,lidar_names))
            print('Start of decision collision state: ' + str(self.collision))
            
            if(self.collision == False):
                forward = (not np.any(booleanArray))
                print('going forward collision state: ' + str(self.collision))
                startTime = time.time()
                while(forward == True):
                    self.goForward()
                    booleanArray,leftHalf_booleanArray,rightHalf_booleanArray,narrow_view = self.booleanArray_parser(self.scan(vehicle_name,lidar_names))
                    # print(narrow_view[int(len(narrow_view)/4):int(len(narrow_view)*3/4)])
                    # forward = (not np.any(booleanArray)) # if narrow view is 0's cleared to go
                    forward = not self.checkForCollision(self.scan(vehicle_name,lidar_names))
                    if((time.time() - startTime) >= 5):
                        forward = False
                # went forward until collision
                self.collision = True
                print('collision detected')
                self.client.hoverAsync().join()
                print('confirmed collision state: ' + str(self.collision))
            else: # self.collision == True
                print('rotating to goal collision state: ' + str(self.collision))
                #collision event
                # turn to goal
                self.turnTowardsGoal(vehicle_name, lidar_names)#synchronous
                # Refresh view        
                overall_point_list = self.scan(vehicle_name,lidar_names)
                booleanArray,leftHalf_booleanArray,rightHalf_booleanArray,narrow_view = self.booleanArray_parser(overall_point_list)
                thetaOffset = 0

                self.collision = (np.any(narrow_view) == True) # check if still in collision [111111111]
                

                
                print('Need to turn right?: ' + str(self.collision))
                # if collision still in the way of goal, try turning right
                if(self.collision == True):
                    turnRight = True
                    while (self.collision == True): #until narrow view all 0's clear to go [0000000]
                        overall_point_list = self.scan(vehicle_name,lidar_names)
                        halftest = (len(overall_point_list))
                        booleanArray,leftHalf_booleanArray,rightHalf_booleanArray,narrow_view = self.booleanArray_parser(overall_point_list)
                        if(self.collision == False):
                            break
                        
                        print(booleanArray)
                        print('Rotating for clear view')
                        self.client.rotateToYawAsync(angleToGoal + thetaOffset, timeout_sec=0.1, margin=0.1, vehicle_name=vehicle_name)
                        thetaOffset = thetaOffset + 0.1

                        booleanArray,leftHalf_booleanArray,rightHalf_booleanArray,narrow_view = self.booleanArray_parser(overall_point_list)
                        if((not np.any(booleanArray)) == True): # if narrow view is all 0's cleared to go [0000000]
                            print('Stop turning we are good to go')
                            turnRight = False
                            self.collision = False
                            print('collision state: ' + str(self.collision))
                            break
                else:
                    self.collision = False
                    pass
                        
            

            # if (((np.all(narrow_view)) == True) or (self.collision == True)):    # if narrow view has any 1's, collision event
            #     # try turning towards goal

                    

            #     # Refresh view        
            #     overall_point_list = self.scan(vehicle_name,lidar_names)
            #     booleanArray,leftHalf_booleanArray,rightHalf_booleanArray,narrow_view = self.booleanArray_parser(overall_point_list)
            #     thetaOffset = 0

            #     self.collision = (not np.any(narrow_view))
                

            #     self.turnTowardsGoal(vehicle_name, lidar_names)#synchronous
            #     # if collision still in the way of goal, try turning right
            #     while ((np.any(narrow_view)) == True): #until narrow view all 0's clear to go
            #         overall_point_list = self.scan(vehicle_name,lidar_names)
            #         halftest = (len(overall_point_list))
            #         booleanArray,leftHalf_booleanArray,rightHalf_booleanArray,narrow_view = self.booleanArray_parser(overall_point_list)
                    
            #         print(narrow_view)
            #         print('Rotating for clear view')
            #         self.client.rotateToYawAsync(angleToGoal + thetaOffset, timeout_sec=0.1, margin=0.1, vehicle_name=vehicle_name)
            #         thetaOffset = thetaOffset + 0.1

            #         booleanArray,leftHalf_booleanArray,rightHalf_booleanArray,narrow_view = self.booleanArray_parser(overall_point_list)
            #         if(np.any(narrow_view) == False): # if narrow view is 0's cleared to go
            #             self.collision = False
            #             break
            # else:
            #     pass
            
            
            # self.client.rotateToYawAsync(angleToGoal + thetaOffset, timeout_sec=0.1, margin=0.1, vehicle_name=vehicle_name)
            #  thetaOffset = thetaOffset + 0.1


                
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


#1. if row all 0's - turn to goal
#2. Go forward until collision
#3. When collision - turn to goal
#4.                 - if still in collision - rotate to right until middle x points 0's
#5. Go forward until collision