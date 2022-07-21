# =============================================================================
# Created By: Tyler Fedrizzi
# Authors: Tyler Fedrizzi
#          Avi Dube
#          Josh Chang
# Created On: September 6th, 2020
# Last Modified: March 5th, 2022
# 
# Description: Module for avoiding obstacles in an agents path
# =============================================================================
import setup_path
import airsim
import logging
import time
import traceback
import math
import json
from utils.oa_utils import Slope

from datetime import datetime
from queue import Queue
from multiprocessing.queues import Empty
from multiprocessing import Process

from utils.data_classes import MovementCommand, VelVec3, PosVec3
from utils.killer_utils import GracefulKiller
from sensors.airsim_lidar import AirSimLiDAR, LiDARBase
# TODO Create status library


class ObstacleAvoidance(Process):
    """
    Module to provide obstacle avoidance systems for the UAV agents to
    be able to fly through their environment.

    ## Inputs:
    - object_queue [Queue] Queue to transmit detected objects data to
                           other modules.
    - lidar_queue [Queue] Queue to transmit pointcloud data to other
                          modules.
    - image_queue [Queue] Queue to transmit image data to other
                          modules.
    - path_planning_queue [Queue] Queue to send movement commands to
                                  move the agent.
    - drone_id [string] Unique identifier for the UAV
    - user_options [dict] User-defined options for path planning
    - simulation [bool] Determines whether we are running the model with
                        AirSim or not
    """

    def __init__(self,
                 path_planning_queue: Queue,
                 drone_id: str,
                 lidar_id: str,
                 sensor_choice: dict,
                 user_options: dict = None,
                 simulation: bool = False):
        # Defines the overall Process class and turns on health listener
        super().__init__(self)
        # This is a section of the global map defined by some radius
        # from the drone
        self.local_map = None
        self.drone_id = str(drone_id)
        self.sensor_name = lidar_id
        # self.object_queue = object_queue
        # self.image_queue = image_queue
        # self.lidar_queue = lidar_queue
        self.path_planning_queue = path_planning_queue
        self.simulation = simulation
        self.global_map = None
        # Allows for global
        # Use the pre-built map and then use the FOV of the sensors to
        # user map so that they can use it.
        # Client to communicate with AirSim
        self.airsim_client = None
        self.publish_velocity = False
        # TODO Add status updates
        self.status = None
        self.last_command = None
        self.takeoff_completed = False
        # TODO Add an inter-process queue between mapping and self
        self.sensor = self.choose_base_sensor(sensor_choice)
        self.algorithm = Slope(5.0, 80.0)

    def build_airsim_client(self):
        """
        Generate the AirSim multirotor client, arming the vehicle
        and enabling API control so that the vehicle is ready to take
        commands from the user.

        ## Inputs:
        - None

        ## Outputs:
        - Assigns the multirotor client to the class airsim_client
        variable.
        """
        self.airsim_client = airsim.MultirotorClient()
        self.airsim_client.confirmConnection()

    def choose_base_sensor(self, sensor_choice: dict) -> LiDARBase:
        """
        Given the User Options, choose which Lidar sensor we are using.

        TODO Find a way to standardize this and/or access a database for
        this information.

        ## Inputs:
        - sensor_choice [dict] The choices made by the User. This
                               contains the "method" parameter and the
                               "hardware" parameter.
        
        Currently, the selection choices are:
        - method -> "AirSim"
        - hardware -> "Velodyne Puck"

        ## Output:
        - None
        """
        if sensor_choice["method"] == "AirSim":
            return AirSimLiDAR(sensor_name=self.sensor_name,
                                      sensor_type=sensor_choice["hardware"],
                                      sensor_position=[0.0,0.0,-0.35],
                                      agent_id=self.drone_id)
        else:
            exit()
    
    def run(self):
        killer = GracefulKiller()

        if self.simulation:
            self.build_airsim_client()
        # TODO Make this a real sensor interface library to connect to
        # different sensors
        while not self.takeoff_completed:
            try:
                message = self.path_planning_queue.get(block=False)
                if message == "Takeoff Completed":
                    self.takeoff_completed = True
                   
            except Exception:
                pass
        try:
            reportTime = time.time()
            while not killer.kill_now:

                data = self.sensor.get_lidar_data(self.airsim_client)
                point_cloud = self.sensor.process_data(data.point_cloud)             
                next_point = self.algorithm.run(point_cloud)
    
                if self.algorithm.publish_point:
                    print("Next Point: {}".format(next_point))
                    command = MovementCommand(
                        position=PosVec3(
                            X=next_point[0],
                            Y=next_point[1],
                            Z=next_point[2]
                        ),
                        move_by="oa"
                    )
                    self.path_planning_queue.put(command)
                # Run at 20 hertz
                time.sleep(0.05)
        except Exception:
            traceback.print_exc()
