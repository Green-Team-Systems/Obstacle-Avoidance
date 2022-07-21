import math
from airsim.client import MultirotorClient
from airsim.types import LidarData

from sensors.lidar_base import LiDARBase


class AirSimLiDAR(LiDARBase):
    """
    Sensor class for the AirSim LiDAR system. This class will 
    replicate the features and interactions of dealing with sensor
    interfaces for real-world LiDAR systems.

    ## Inputs:
    - sensor_type [str] The type of LiDAR you are using
    - sensor_name [str] Unique name of the sensor
    - sensor_position [list] The X, Y, Z position of the sensor in NED
    """
    def __init__(self,
                 sensor_name: str,
                 sensor_type: str,
                 sensor_position: list,
                 agent_id: str,
                 update_rate: int = 10) -> None:
        super().__init__(sensor_name=sensor_name,
                         sensor_type=sensor_type,
                         sensor_position=sensor_position)
        # This will provide access to the following parameters:
        # - Range
        # - Number of Channels
        self.update_rate = update_rate
        self.agent_id = agent_id
    
    def process_data(self, point_cloud: list) -> list:
        """
        Given a flat array of floats, interpret the point cloud as a 
        2-D matrix to utilize for obstacle avoidance, mapping and more.

        ## Inputs:
        - point_cloud [list] a list of float values that contain the 
                             x, y and z coordinates of a point returned
                             by the LiDAR system. The points are in the 
                             SensorLocalFrame, in the NED coordinate
                             framework.
        """ 
        if (len(point_cloud) < 3):
            return list()

        # Divides the lidarData into 3 seperate variables for x, y and z
        y_points_last = 0.0
        length = len(point_cloud)
        overall_point_list = list()
        next_row = list()
        for i in range(0, length, 3):
            xyz = point_cloud[i:i+3]
            # If we have reached the right-hand edge of a row, go to the
            # next row
            if (xyz[1] != math.fabs(xyz[1])
                and y_points_last != 0.0
                and y_points_last == math.fabs(y_points_last)):
                overall_point_list.append(next_row)
                next_row = list()
            next_row.append(xyz)
            y_points_last = xyz[1]

        return overall_point_list

    def get_lidar_data(self, airsim_client: MultirotorClient) -> LidarData:
        """
        Query AirSim for the results of the Lidar system

        ## Inputs:
        - airsim_client [MultirotorClient] the API client to query the
                                           AirSim library.
        """
        lidar_data = airsim_client.getLidarData(lidar_name=self.sensor_name,
                                                vehicle_name=self.agent_id)
        
        return lidar_data
