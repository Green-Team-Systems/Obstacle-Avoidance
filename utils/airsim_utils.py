# =============================================================================
# Copyright 2022. Codex Laboratories LLC
#
# Created By: Tyler Fedrizzi
# Created On: 9 June 2022
#
#
# Description: AirSim interface file for the SWARM platform
# =============================================================================
import logging
import setup_path
import airsim
import time

from threading import Thread, Event
from datetime import datetime
from airsim.types import Quaternionr, YawMode

from utils.data_classes import Orientation, PosVec3
from utils.position_utils import position_to_list, quaternion_to_yaw
from utils.position_utils import gps_velocity_to_list, gps_position_to_list
from utils.position_utils import vector_to_list


class ConnectorBase():
    """
    Base connector class for connecting to the Vehicle data ports. This
    class defines a set of methods for MavLink, AirSim, Gazebo and other
    systems.
    """
    def __init__(self) -> None:
        # Thread.__init__(self, daemon=True)
        self.shutdown_system = Event()
        self.takeoff_completed = Event()

    def takeoff(self) -> bool:
        pass

    def land(self) -> bool:
        pass

    def update_pose(self) -> None:
        """
        Virtual method to be defined by child class. May or may not be
        used depending on the current system of use
        """
        pass

    def propagate_pose(self) -> None:
        pass

    def shutdown(self) -> None:
        pass

    def run(self) -> None:
        pass


class PoseAirSimConnector(ConnectorBase):
    """
    A class defining the data connections between AirSim and the SWARM
    agent. This class abstracts from the internal system the data calls
    that are required for the autonomy to function properly with the
    AirSim system.

    # Inputs:
    - None
    """
    def __init__(self,
                 starting_position: PosVec3,
                 starting_orientation: Orientation,
                 log: logging.Logger,
                 drone_id: str,
                 data_rate: int = 10) -> None:
        super().__init__()
        self.starting_position = starting_position
        self.starting_orientation = starting_orientation
        # All agents start from their origin point (0,0,0) in NED
        self.current_position = PosVec3()
        self.current_orientation = Orientation()
        self.log = log
        self.airsim_client = None
        self.drone_id = drone_id
        self.data_rate = data_rate

    def build_airsim_client(self) -> None:
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
        self.airsim_client.armDisarm(True, self.drone_id)
        self.airsim_client.enableApiControl(True, self.drone_id)
        self.log.info("{}|{}|message|{}".format(
                                datetime.utcnow(),
                                self.drone_id,
                                "AirSim API connected!"
                                )
                            )

    def takeoff(self) -> bool:
        """
        Command the UAV to takeoff by making the proper Async call to
        AirSim.
        The client call returns a future and we wait for that future to
        resolve until we continue execution.

        ## Inputs:
        - None

        ## Outputs:
        - Informs the path planning module that the UAV is now
        airborne.
        """
        droneTakeoff = self.airsim_client.takeoffAsync(
                vehicle_name=self.drone_id)
        droneTakeoff.join()
        
        self.takeoff_completed.set()
        self.log.info("{}|{}|message|{}".format(
                                datetime.utcnow(),
                                self.drone_id,
                                "Takeoff Complete! System is airborne!"
                                )
                            )
        return True
    
    def roll_pitch_yaw(self, orientation: Quaternionr):
        print(orientation)
        pitch, roll, yaw  = airsim.to_eularian_angles(orientation)

        return [roll, pitch, yaw]

    def update_pose(self) -> None:
        state_data = self.airsim_client.getMultirotorState(
            vehicle_name=self.drone_id)
        self.local_position = position_to_list(
            state_data.kinematics_estimated.position, frame="local")
        self.global_position = position_to_list(
            state_data.kinematics_estimated.position,
            starting_position=self.starting_position,
            frame="global")
        self.heading = quaternion_to_yaw(state_data.kinematics_estimated.orientation)
        self.current_orientation = Orientation(yaw=self.heading)
        self.gps_pos_vec3 = gps_position_to_list(
            state_data.gps_location)
        self.current_velocity = gps_velocity_to_list(
            state_data.kinematics_estimated.linear_velocity)
        self.drone_velocity = self.current_velocity.toPythonList()
        self.drone_position = self.global_position.toPythonList()
        self.drone_gyro = vector_to_list(state_data.kinematics_estimated.angular_acceleration)
        self.drone_attitude = self.roll_pitch_yaw(state_data.kinematics_estimated.orientation)
        print("Drone Attitude: {}".format(self.drone_attitude))

    def shutdown(self) -> None:
         self.airsim_client.armDisarm(False, vehicle_name=self.drone_id)
         self.airsim_client.enableApiControl(
            False,
            vehicle_name=self.drone_id
         )
         self.log.info("{}|{}|message|{}".format(
                                datetime.utcnow(),
                                self.drone_id,
                                "Vehicle disarmed and disconnected!"
                                )
                            )

    def velocity_command(self,
                         xVel: float,
                         yVel: float,
                         zVel: float,
                         speed: float,
                         heading: float) -> None:
        self.airsim_client.moveByVelocityAsync(
                    xVel,
                    yVel,
                    zVel,
                    speed,
                    yaw_mode=YawMode(False, heading),
                    vehicle_name=self.drone_id
                )

    def acceleration_command(self,
                             roll: float,
                             pitch: float,
                             yaw: float,
                             throttle: float,
                             heading: float) -> None:
        self.airsim_client.moveByAngleRatesThrottleAsync(
                roll_rate=roll,
                pitch_rate=pitch,
                yaw_rate=yaw,
                throttle=throttle,
                duration=1.0,
                vehicle_name=self.drone_id
            )

    def yaw_command(self,
                    heading: float,
                    timeout: float,
                    margin: float) -> None:
        self.airsim_client.rotateToYawAsync(heading,
                                            timeout_sec=timeout,
                                            margin=margin,
                                            vehicle_name=self.drone_id)

    def run(self) -> None:
        """
        Main function to draw state data from Unreal Engine at a
        specific rate designated by the user.

        # Inputs:
        - None

        # Outputs:
        - None
        """
        self.build_airsim_client()

        while not self.takeoff_completed.is_set():
            pass

        while not self.shutdown_system.is_set():
            self.update_pose()
            self.propagate_pose()
            time.sleep(1 / self.data_rate)
        
        self.shutdown()
