# =============================================================================
# Created By: Tyler Fedrizzi
# Authors: Tyler Fedrizzi
# Created On: September 6th, 2020
# Last Modified: Septebmer 5th, 2021
# 
# Description: Module for avoiding obstacles in an agents path
# =============================================================================

from os import kill
import setup_path
import airsim
import logging
import time
import copy
import traceback

from multiprocessing import Process
from datetime import datetime
from multiprocessing.queues import Empty

from utils.data_classes import PosVec3, MovementCommand
from utils.killer_utils import GracefulKiller
from airsim.types import YawMode
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
                 path_planning_queue,
                 drone_id,
                 user_options=None,
                 simulation=False):
        Process.__init__(self, daemon=True)
        # This is a section of the global map defined by some radius
        # from the drone
        self.local_map = None
        self.drone_id = str(drone_id)
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
        # TODO Add status updates
        self.status = None
        self.last_command = None
        self.takeoff_completed = False
        # TODO Add an inter-process queue between mapping and self
        FORMAT = '%(asctime)s %(message)s'
        # logging.basicConfig(format=FORMAT,
         #                   level=logging.INFO)
        file_handler = logging.FileHandler(
            "logs/{drone_id}-{name}-{date}.log".format(
                name=__name__,
                drone_id=self.drone_id,
                date="{}-{}-{}".format(
                    datetime.utcnow().month,
                    datetime.utcnow().day,
                    datetime.utcnow().year,
                )
        ))
        self.log = logging.getLogger(self.drone_id + __name__)
        self.log.setLevel(logging.INFO)
        self.log.addHandler(file_handler)
    
    def start(self):
        Process.start()
    
    def run(self):
        Process.run(self)
        killer = GracefulKiller()
        if self.simulation:
            self.build_airsim_client()
        while not self.takeoff_completed:
            try:
                message = self.path_planning_queue.get(block=True, timeout=0.0)
                if message == "Takeoff Completed":
                    self.takeoff_completed = True
                # TODO Handle takeoff failures
            except Exception:
                pass
        try:
            while not killer.kill_now:
                pass
        except Exception as error:
            self.log.error(traceback.print_exc())