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

from os import kill
from turtle import heading
import setup_path
import airsim
import logging
import time
import copy
import traceback
import math
import json

from multiprocessing import Process
from datetime import datetime
from multiprocessing.queues import Empty

from utils.data_classes import PosVec3, MovementCommand, VelVec3, OASystemStates
from utils.position_utils import position_to_list
from utils.killer_utils import GracefulKiller
from airsim.types import LidarData, YawMode
from walltrace import WallTrace
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
                 lidar_id,
                 user_options=None,
                 simulation=False):
        Process.__init__(self, daemon=True)
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
        self.state = OASystemStates.GROUNDED
        # TODO Add an inter-process queue between mapping and self
        

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
        # self.airsim_client.armDisarm(True, self.drone_id)
        # self.airsim_client.enableApiControl(True, self.drone_id)

    def start(self):
        Process.start(self)
    
    def run(self):
        Process.run(self)
        FORMAT = '%(asctime)s|%(message)s'
        logging.basicConfig(format=FORMAT,
                            level=logging.INFO)
        file_handler = logging.FileHandler(
            "logs/{drone_id}-{name}-{date}-Obstacle-Avoidance.log".format(
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
        killer = GracefulKiller()
        if self.simulation:
            self.build_airsim_client()
        while not self.takeoff_completed:
            try:
                message = self.path_planning_queue.get(block=False)
                if message == "Takeoff Completed":
                    self.takeoff_completed = True
                    self.state = OASystemStates.PATH_PLANNING
                    self.log.info(
                                "{}|{}|distances|{}".format(
                                    datetime.utcnow(),
                                    self.drone_id,
                                    "Takeoff Completed. Starting sensing!"
                                    )
                                )
            except Exception:
                pass
        try:
            while not killer.kill_now:
                lidar_data = self.airsim_client.getLidarData(
                    lidar_name=self.sensor_name,
                    vehicle_name=self.drone_id
                )
                drone_state = self.airsim_client.getMultirotorState(
                vehicle_name=self.drone_id
                )
                position = position_to_list(
                    drone_state.kinematics_estimated.position
                )
                data = lidar_data.point_cloud
                #Call Harrisons Algo here
                if(self.state != OASystemStates.CLEARANCE):
                    if(position.Z < 120):
                        angle, x_vel, y_vel, z_vel = WallTrace.execute(data) #run Josh algo
                        print(x_vel)
                        if x_vel != 0 and angle != 0:
                            self.state = OASystemStates.WALLTRACE
                        else: 
                            self.state = OASystemStates.PATH_PLANNING
                    else:
                        x_vel, z_vel = self.slopeCalculation(data, 10.0) # calls the slope calculation method
                # TODO Add state switching logic
                if self.state == OASystemStates.SLOPE:
                    self.log.info(
                            "{}|{}|vel_command|{}".format(
                                datetime.utcnow(),
                                self.drone_id,
                                json.dumps([x_vel,z_vel])
                                )
                            )
                    command = MovementCommand(
                        velocity=VelVec3(
                            vx=x_vel,
                            vz=-1 * z_vel # remeber that z is in ned corrdinate system so we have to inverse the speed of z velocity 
                        ),
                        move_by="slope" # creates a move by velocity command for the planner to use in deciding who controls the drone
                    )
                    self.path_planning_queue.put(command)
                elif self.state == OASystemStates.WALLTRACE:
                    self.log.info(
                            "{}|{}|vel_command|{}".format(
                                datetime.utcnow(),
                                self.drone_id,
                                json.dumps([x_vel,y_vel,angle])
                                )
                            )
                    command = MovementCommand(
                            velocity=VelVec3(
                                vx=x_vel,
                                vy=y_vel,
                                vz=-1 * z_vel 
                            ),
                            heading = angle,
                            move_by="velocity" # creates a move by velocity command for the planner to use in deciding who controls the drone
                        )
                    self.path_planning_queue.put(command)
                # Run at 20 hertz
                time.sleep(0.04)
        except Exception as error:
            traceback.print_exc()
            self.log.info(f"There was an error: {error}")

    def slopeCalculation(self, lidarData, droneVelocity):
        xVelocity = 0.0
        zVelocity = 0.0
    # Depending on the range of the Lidar sensor (in the settings.json) no points will be recieved if the points distance exceeds the range.
        if (len(lidarData) < 3):
            self.log.info(
                            "{}|{}|message|{}".format(
                                datetime.utcnow(),
                                self.drone_id,
                                "No Data Received"
                                )
                            )
            self.publish_velocity = False
            return xVelocity, zVelocity
            # Intended to Unrotate after a rotation was completed to avoid collision.
            # Currently it rotates to a static Yaw value and will need to be adjusted for relative values
            # Divides the lidarData into 3 seperate variables for x, y and z
        start = time.time()
        y_points_last = 0
        length = len(lidarData)

        overall_point_list = list()
        next_row = list()

        for i in range(0, length, 3):
            xyz = lidarData[i:i+3]
            if (xyz[1] != math.fabs(xyz[1]) and y_points_last == math.fabs(y_points_last)):
                overall_point_list.append(next_row)
                next_row = list()
            next_row.append(xyz)
            y_points_last = xyz[1]
        try: 
            
            midpoint_top_level = int(len(overall_point_list[1]) / 2)
            x2_distance = overall_point_list[1][midpoint_top_level][0]
            z2_distance = -overall_point_list[1][midpoint_top_level][2]
            bottom_level_point = len(overall_point_list) - 1
            midpoint_bottom_level = int(len(overall_point_list[bottom_level_point]) / 2)
            x1_distance = overall_point_list[bottom_level_point][midpoint_bottom_level][0]
            z1_distance = -overall_point_list[bottom_level_point][midpoint_bottom_level][2]

            # If we are within our distance threshold
            if x2_distance < 11.0 or x1_distance < 3.0 or z1_distance > -1.5:

                x_distance = math.fabs(x2_distance - x1_distance)
                z_distance = math.fabs(z2_distance - z1_distance)

                hypo = math.sqrt(math.pow(x_distance, 2) + math.pow(z_distance, 2)) 
                # Go at the maximum speed in the upward direction but scale by
                # the distance been the two z points. As we get closer to the
                # ground, increase the Z /velocity.
                zVelocity = (z_distance / hypo) * 16.0 * (1 / x_distance) # Avoidance velocity
                xVelocity = (x_distance / hypo) * -1

            # print(f'Z speed: {zVelocity}')
            # print(f'X speed: {xVelocity}')
            # print(f'X distance: {x_distance}')
            # print(f'Z distance: {z_distance}')

        except Exception:
            traceback.print_exc()
            self.publish_velocity = False
            return xVelocity, zVelocity

        if zVelocity != 0.0:
            self.publish_velocity = True
            self.state = OASystemStates.SLOPE
        else:
            self.publish_velocity = False

        self.log.info(
                      "{}|{}|message|{}".format(
                        datetime.utcnow(),
                        self.drone_id,
                        "Processing Time: {}".format(time.time() - start)
                        )
                     )
        return xVelocity, zVelocity

