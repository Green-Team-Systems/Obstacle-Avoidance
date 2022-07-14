# ===============================================================
# Copyright 2022. Codex Laboratories LLC
# Created By: Tyler Fedrizzi
# Authors: Tyler Fedrizzi
# Created On: July 14th, 2022
# Updated On: July 14th, 2022
# 
# Description: Algorithms and utilities for obstacle avoidance
# ===============================================================
import math

PRECISION = 4


class Algorithm():
    """
    Base algorithm class utilized to provide common features and memory
    space to individual algorithms.

    ## Inputs:
    - None
    """
    def __init__(self):
        self.maximum_memory = 100.0  # Megabytes

    def run(self):
        raise NotImplementedError("Please implement this method!")


class Slope(Algorithm):
    """
    Algorithm that determines the slope of the terrain in front of it
    and then calculates a new point to travel to the slope.

    ## Inputs:
    - min_incline (float) the minimum slope in degrees to turn on the
                          algorithm
    - max_incline (float) the maximum slope in degrees to utilize the
                          algorithm before switching to a different
                          method.
    - sensor_view_angle (float) the view angle of the camera, as
                                measured by the centerline of the
                                vehicle and measured in the X, Z plane,
                                with Z being downards in the NED system
    """
    def __init__(self, min_incline: float,
                 max_incline: float,
                 sensor_view_angle: float = 30.0,
                 debug: bool = False):
        super().__init__()
        self.min_incline = min_incline  # Degrees
        self.max_incline = max_incline  # Degrees
        self.previous_slope = 0.0
        self.publish_point = False
        self.sensor_view_angle = sensor_view_angle  # Degrees
        self.ground_clearance = 3.0  # Meters
        self.debug = debug

    def run(self, point_cloud: list) -> float:
        """
        Given a 2D matrix of (x,y,z) points, determince the calculated
        slope of the given terrain.

        ## Inputs:
        - point_cloud [list[list]] A list of NUMB_CHANNELS lists, where
                                    each list represents 1 row of
                                    detected points. Each point is a list
                                    of (x,y,z) float values in the sensor
                                    frame (NED).

        ## Outputs:
        - 
        """
        # Depending on the range of the Lidar sensor (in the settings.json)
        # no points will be recieved if the points distance exceeds the range.
        if (len(point_cloud) < 2.0):
            self.publish_point = False
            return None

        try:             
            midpoint_top_level = int(len(point_cloud[0]) / 2)
            x2_distance = point_cloud[0][midpoint_top_level][0]
            z2_distance = -point_cloud[0][midpoint_top_level][2]
            bottom_level_point = len(point_cloud) - 1
            midpoint_bottom_level = int(len(point_cloud[bottom_level_point]) / 2)
            x1_distance = point_cloud[bottom_level_point][midpoint_bottom_level][0]
            z1_distance = -point_cloud[bottom_level_point][midpoint_bottom_level][2]

            # TODO Sample middle point and establish how steep the hill is
            # TODO Project current path moving forward up and forward of the
            # path of motion of the vehicle.
            # TODO Push back point in cleared space to go

            x_distance = math.fabs(x2_distance - x1_distance)
            z_distance = math.fabs(z2_distance - z1_distance)

            # Given the distances, do rise over run
            slope = math.atan2(z_distance, x_distance)  # Radians
            self.previous_slope = slope

            if self.debug:
                print("Calculated slope is {}".format(math.degrees(slope)))
            # If we an increasing slope greater then the minimum
            if math.degrees(slope) > self.min_incline and math.degrees(slope) <= self.max_incline:
                next_point = point_cloud[0][midpoint_top_level]  # (X, Y, Z) in NED [meters]
                if self.debug:
                    print(f"Next Point before Clearnace: {next_point}")
                next_point[2] -= self.ground_clearance  # Go up in the down Z

                next_point = [round(dim, PRECISION) for dim in next_point]

                self.publish_point = True
                return next_point
            elif math.degrees(slope) > self.max_incline:
                pass
            else:
                # The ground is flat, so move forward
                self.publish_point = False
                return None

        except Exception as error:
            if self.debug:
                print(f"Error: {error}")
            self.publish_point = False
            return None

        self.publish_point = False
        return None

    """
    def moveForward(self):
        self.client.moveByVelocityAsync(6, 0, 0, math.inf) 
    
    def moveRight(self):
        self.client.moveByVelocityAsync(0, 2, 0, math.inf) 

    def simple_right(self, lidarData):
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
            # return [0, 1, 0] for later use with trajectory planning 
        else:
            self.moveForward()
            # return [1, 0, 0]
    """