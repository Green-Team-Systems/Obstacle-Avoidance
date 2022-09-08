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
import numpy as np

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


class RightHandRule(Algorithm):

    def __init__(self):
        super().__init__()

    def moveForward(self):
        # TODO Use the fact that we have the point we detected
        self.client.moveByVelocityAsync(6, 0, 0, math.inf) 
    
    def moveRight(self):
        # TODO Use the fact that we have the point detected
        self.client.moveByVelocityAsync(0, 2, 0, math.inf) 

    def run(self, point_cloud: list):
    # Depending on the range of the Lidar sensor (in the settings.json) no points will be recieved if the points distance exceeds the range.
        #state_data = self.client.getMultirotorState()
        
        top_level = point_cloud[0]
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
            
class WallTrace(Algorithm):
    STATES = {
        "Clear": "TOGOAL",
        "Obstacle": "AVOID"
        }
    def __init__(self, view_distance: float):
        super().__init__()
        self.view_distance = view_distance
        self.mode = self.STATES["Clear"]

    """
    Description: Find closest row parallel to the z level of the drone
    
    Inputs: overall_point_list
    
    Outputs: row index i
    
    Notes:
    """
    def chooseRow(self, overall_point_list):
        top_row = overall_point_list[1] 
        mid_point_top_level_ind = int(len(top_row) / 2)
        z_level_row = top_row[mid_point_top_level_ind][2] # initial z val of top row mid
        i = 1

        for row_index, row in enumerate(overall_point_list):
            if(len(row) > 0):
                mid_point = int(len(row) / 2) # index of mid point
                mid_point_z = row[mid_point][2] # z value of mid point
                if(abs(mid_point_z) < abs(z_level_row)):
                    z_level_row = mid_point_z

                    i = row_index

        return i

    """
    Description: detects gaps between points and returns row of lidar points with a gap symbol
    
    Inputs: threshold - distance between points, row of points
    
    Outputs: Fixed row of points

    """
    def dataFixer(self, threshold, chosen_row):

        #Todo: place 'Gap' at the end / corner
        last_point = np.array([chosen_row[0][0], chosen_row[0][1]])  #list of x and y value of first point
        for ind, point_val in enumerate(chosen_row):
            temp = last_point - np.array([point_val[0], point_val[1]])
            vector_dist = np.sqrt(temp[0]**2 + temp[1]**2)

            #calculated the vector distance between two points to see if there's a gap in the lidar data
            if vector_dist > threshold:
                chosen_row.insert(ind, 'G')
            last_point = np.array([point_val[0], point_val[1]])

        return chosen_row
        
    """
    Description: Filter lidar data by a distance threshold
    
    Inputs: view distance set, row of lidar points
    
    Outputs: filtered row of lidar points
    """
    def view_distance_filter(self, fixed_row, view_distance = 5):
        #Todo: make function consider 'G'
        filtered_row = []
        for ind, val in enumerate(fixed_row):
            if val != 'G':
                dist = val[0]
                if(dist <= view_distance):
                    filtered_row.append(val)


        return filtered_row
    
    """
    Description: Calculate the sum of the vectors from left to right
    
    Inputs: row of lidar points 
    
    Outputs: sum vector
    """
    def calculate_object_sum_vector(self, fixed_row):
        #find left-most group of points between gaps

        first_point = fixed_row[0]
        sum_vector = [0, 0]
        for ind, point in enumerate(fixed_row):
            #this will break if a G is at the start of the row
            if point == 'G':
                break
            
            vector = [(point[0] - first_point[0]), (point[1] - first_point[1])]
            sum_vector = [sum_vector[0] + vector[0], sum_vector[1] + vector[1]]
            
        return sum_vector    
    
    def normalize_vector(self, vector):
        
        try:
            vx = vector[0] / math.sqrt(vector[0]**2 + vector[1]**2)
            vy = vector[1] / math.sqrt(vector[0]**2 + vector[1]**2)
            normalized_vector = [vx, vy]
        except ZeroDivisionError:
            return [0,0]

        return normalized_vector
    
    """
    Description: Calculates the offset angle (45 deg) from the wall vector
    
    Inputs: wall vector
    
    Outputs: Offset wall vector
    """
    def vector_45_from_wall(self, wall_vector):
        #rotates vector by 45 degrees
        x_component = float(-1) * (math.cos(math.sqrt(2)/2) * wall_vector[0]) - (math.sin(math.sqrt(2)/2) * wall_vector[1])
        y_component = (math.sin(math.sqrt(2)/2) * wall_vector[0]) + (math.cos(math.sqrt(2)/2)* wall_vector[1])
        offwallvector = [x_component, y_component]
        
        return offwallvector    
    
    # Avoidance functions
    def avoid(self, fixedchosenRow):
        # turn 45 against wall
        # strafe with wall vector
        # break out when filtered view is clear

        filtered_row = self.view_distance_filter(fixedchosenRow, self.view_distance)
        # get vector 45 degrees from wall
        if (filtered_row is None):
            return
        sum_vector = self.calculate_object_sum_vector(fixedchosenRow)
        norm_sum_vector = self.normalize_vector(sum_vector)
        if(norm_sum_vector is [0,0]):
            return
        vectorfromwall = self.vector_45_from_wall(norm_sum_vector)
        # get angle from drone to wall vector
        # angleInRad,angleInDegree = self.angle_from_drone_to_vector(vectorfromwall)

        # TODO: how to pass yaw_mode parameter
        x_Vel = norm_sum_vector[0]
        y_Vel = norm_sum_vector[1]
        z_Vel = 0
        

        # return angleInDegree, x_Vel, y_Vel, z_Vel   
        return x_Vel, y_Vel, z_Vel  
    
    def run(self, point_cloud: list):
        #choose the row nearest to z = 0 (relative to drone level)
        chosenRowIndex = self.chooseRow(point_cloud)
        #correct for gaps in data (if no wall is behind, lidar will omit any gaps)
        fixedchosenRow = self.dataFixer(1, point_cloud[chosenRowIndex])
        # filter lidar array to get only the points within a certain distance
        filtered_row = self.view_distance_filter(fixedchosenRow, self.view_distance)
        
        # Flight mode switch
        if (filtered_row != [] and self.mode == self.STATES["Clear"]):
            self.mode = self.STATES["Obstacle"]
        elif (filtered_row == [] and self.mode == self.STATES["Obstacle"]):
            self.mode = self.STATES["Clear"]
        
        # execution of the current flight mode
        if self.mode == "TOGOAL":
            # turn to destination coordinates on the xy plane
            x_Vel = 0
            y_Vel = 0
            z_Vel = 0
            angleInDegree = 0
        elif self.mode == "AVOID":
            x_Vel, y_Vel, z_Vel = self.avoid(fixedchosenRow)
        
        next_point = [x_Vel, y_Vel, z_Vel]
        # return angle in degrees, x_Vel, y_Vel, z_Vel
        return next_point