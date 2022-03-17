# =============================================================================
# Created By: Tyler Fedrizzi
# Authors: Tyler Fedrizzi
# Created On: September 6th, 2020
# Last Modified: Septebmer 5th, 2021
# 
# Description: Module for implementing movement commands to the Aircraft
# =============================================================================

from os import X_OK
import traceback
import setup_path
import airsim
import logging
import time
import copy
import numpy as np
import math as m
import json

from multiprocessing import Process
from datetime import datetime
from multiprocessing.queues import Empty

from utils.data_classes import PosVec3, MovementCommand, VelVec3
from utils.killer_utils import GracefulKiller
from airsim.types import YawMode
from utils.distance_utils import ned_position_difference
from utils.position_utils import position_to_list
# TODO Create status library


class PathPlanning(Process):
    """
    A separate process that handles all path planning considerations,
    to include submitting points to the low-level controller, handling
    tracking of the current position and ensuring that all interactions
    are being handled appropriately.

    ## Inputs:
    - queue [Queue] Transmission queue to communicate with main process.
    - drone_id [string] Unique identifier for the UAV
    - user_options [dict] User-defined options for path planning
    - simulation [bool] Determines whether we are running the model with
                        AirSim or not
    """

    def __init__(self, queue, drone_id, user_options=None, simulation=False):
        Process.__init__(self, daemon=True)
        # This is a section of the global map defined by some radius
        # from the drone
        self.local_map = None
        self.drone_id = str(drone_id)
        self.command_queue = queue
        self.simulation = simulation
        # Determining whether we are in the Air or Not
        self.airborne = False
        self.global_map = None
        # Allows for global
        # Use the pre-built map and then use the FOV of the sensors to
        # user map so that they can use it.
        # Client to communicate with AirSim
        self.airsim_client = None
        # TODO Add status updates
        self.status = None
        self.last_command = MovementCommand()
        self.last_velocities = VelVec3()
        self.commands = list()
        self.errors = {
            "X": 0.0,
            "Y": 0.0,
            "Z": 0.0
        }
        self.integral_error = {
            "X": 0.0,
            "Y": 0.0,
            "Z": 0.0
        }
        self.previous_velocities = {
                "VX": 0.0,
                "VY": 0.0,
                "VZ": 0.0,
            }
        self.error_count = 0
        self.last_position = PosVec3()
        self.current_time = datetime.utcnow()
        self.previous_time = datetime.utcnow()
        # TODO Add an inter-process queue between mapping and self
        FORMAT = '%(asctime)s %(message)s'
        # logging.basicConfig(format=FORMAT,
         #                   level=logging.INFO)
        file_handler = logging.FileHandler(
            "logs/{drone_id}-Path-Planning-{date}.log".format(
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
        self.airsim_client.armDisarm(True, self.drone_id)
        self.airsim_client.enableApiControl(True, self.drone_id)
        self.log.info("{}|{}|message|{}".format(
                                datetime.utcnow(),
                                self.drone_id,
                                "AirSim API connected!"
                                )
                            )

    def generate_trajectory(self, point, velocity, characteristics=[]):
        """

        """
        # point (x,y,z)
        # velocity
        # characteristics = Sneaky, Loud, Specific height,
        #                   Agressive vs. Conserative
        # Sneaky - buffer distance
        # Global creates a trajectory to that point
        # Local handles obstacles between your current position and this point
        # Local updates rapidly from the global map
        pass

    def local_path_trajectory(self):
        # Update rate (defined by our sensor)
        pass

    def fly_to_new_position(self, position: PosVec3, velocity: float) -> None:
        """
        Fly to a new position, receiving the position either directly
        from the main intelligence module or as a component of a path
        planning algorithm.

        Inputs:
        - position [PosVec3] Position to move to, given as X, Y and Z
                             coordinates relative to the body coord.
                             frame of the aircraft.
        - velocity [float] Forward velocity of the quad copter.
        """
        # z_coord = abs(position.Z)
        self.airsim_client.moveToPositionAsync(
            position.X,
            position.Y,
            position.Z,
            velocity,
            vehicle_name=self.drone_id)

    def takeoff(self) -> None:
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
        self.airborne = True
        
        self.log.info("{}|{}|message|{}".format(
                                datetime.utcnow(),
                                self.drone_id,
                                "Takeoff Complete! System is airborne!"
                                )
                            )

    def start(self):
        """
        Override Process start function and update status
        """
        Process.start(self)
        # TODO Update status of operations
    
    def check_for_new_command(self, command: MovementCommand) -> bool:
        """
        Compare the current command to the previous command. If the
        commands are the same ignore that command because it was a
        duplicate given by intelligence module due to processing
        constraints.

        TODO Implement some type of execution status that can be
             tracked by the Intel module so we don't get eroneous
             movement commands.
            
        ## Inputs:
        - command [MovementCommand] The next NED local position to
                                    move to.
        ## Outputs:
        Boolean of whether this is a new command or not
        """
        if self.last_command == None:
            return True

        if (command.position.X == self.last_command.position.X
            and command.position.Y == self.last_command.position.Y
            and command.position.Z == self.last_command.position.Z):
            return False
        else:
            return True

    def receive_commands(self) -> list:
        """
        We constantly need to be checking the command queue to see if
        there are any path planning calls that need to be executed by
        UAV. We listen on the queue, in blocking mode when we are not
        doing other computation, and once we receive a new location, we
        execute that method.

        ## Inputs
        - None

        ## Outputs
        A list of PosVec3 local positions to command the UAV to. It
        could be a set of positions that equal a trajectory or just the
        next point. We ensure generality by not being specific about how
        many points.
        """
        # This is not reliable. Ideally, we process this async by firing off
        # a command upon every message receipt. This will be implemented later,
        # maybe, depending on how we feel about this.
        # NOTE This doesn't work on MacOS due to:
        # NOTE https://docs.python.org/3/library/multiprocessing.html#multiprocessing.Queue.qsize

        # Try 5 times to get messages then execute them
        for _ in range(10):
            try:
                self.commands.append(
                    self.command_queue.get(
                    block=True,
                    timeout=0.0
                    )
                )
            except Empty:
                continue

    def run(self):
        """
        Override of the Process run command to define behavior of the
        path planning module.

        Basic workflow:
        1. Check for a command to build a path
        2. Check for a raw position to be given
        3. If building a path:
            3a. Query map and receieve immediate obstacles
            3b. Build trajectory and store
            3c. Inform control trajecotry generated
        4. Execute trajectory or push next location

        TODO Refactor workflow to use ROS
        """
        Process.run(self)
        killer = GracefulKiller()

        if self.simulation:
            self.build_airsim_client()

        if not self.airborne and self.simulation:
            self.takeoff()
        for _ in range(2):
            self.command_queue.put("Takeoff Completed")
        # TODO What if there is a collision or error?

        # Main Execution
        self.log.info("{}|{}|message|{}".format(
                            datetime.utcnow(),
                            self.drone_id,
                            "Beginning main path planning execution"
                        )
                    )
        # self.command_queue.join()
        while not killer.kill_now:
            # Check the queue for commands. Will block until message received.
            # self.receive_commands()
            # TODO Add a common inter-process message to either containerize
            # the underlying set of messages or act as a header message for
            # the stream of messages.
            # TODO Add path planning algorithms for processing
            if self.simulation:
                try:
                    command = self.command_queue.get(
                        block=True,
                        timeout=0.0
                        )
                    self.log.info("{}|{}|commanded_position|{}".format(
                        datetime.utcnow(),
                        self.drone_id,
                        command)
                    )
                    try:
                        assert isinstance(command, MovementCommand)
                        
                        if(self.check_for_new_command(command) == True):
                            startTime = datetime.utcnow()
                        self.move_to_next_position(command, startTime)
                        # TODO Send back message on command completion
                        self.log.info(
                            "{}|{}|commanded_position_completed|{}".format(
                                datetime.utcnow(),
                                self.drone_id,
                                command
                                )
                            )
                        if command.move_by == "position":
                            self.last_command = copy.deepcopy(command)
                    except Exception as error:
                        traceback.print_exc()
                        self.log.error("{}|{}|error|{}".format(
                            datetime.utcnow(),
                            self.drone_id,
                            error)
                        )
                # self.commands.clear()
                except Empty:
                    continue
            else:
                time.sleep(0.1)
        self.log.info("{}|{}|message|{}".format(
                                datetime.utcnow(),
                                self.drone_id,
                                "Shutdown initiated!"
                                )
                            )
        
        # Clean shutdown by disarming and disabling API control
        if self.simulation:
            self.airsim_client.armDisarm(False, vehicle_name=self.drone_id)
            self.airsim_client.enableApiControl(
                False,
                vehicle_name=self.drone_id
            )
            self.airsim_client.reset()

        self.log.info("{}|{}|message|{}".format(
                                datetime.utcnow(),
                                self.drone_id,
                                "Shutdown completed!"
                                )
                            )

    def calculate_velocities(self,target_pos, current_pos, command, startTime):
        kP = 0.4
        kI = 0.0
        kD = 0.2
        vMax = 20 
        jMax = 25
        # print(target_pos.Z, current_pos.Z)
        x_error = target_pos.X - current_pos.X
        y_error = target_pos.Y - current_pos.Y
        z_error = -1 * (target_pos.Z - current_pos.Z)

        # if(self.check_for_new_command(command) == True):
        #     startTime = datetime.utcnow()
        #     print(startTime)
        now = datetime.utcnow()
        slew = (now - startTime).total_seconds()
        dt = (now - self.previous_time).total_seconds()
        self.previous_time = now

        x_derivative = (x_error - self.errors["X"]) / dt
        y_derivative = (y_error - self.errors["Y"]) / dt
        z_derivative = (y_error - self.errors["Y"]) / dt

        x_integral = (self.integral_error["X"] + x_error) * dt
        y_integral = (self.integral_error["Y"] + y_error) * dt
        z_integral = (self.integral_error["Z"] + z_error) * dt
        self.error_count += 1
        if self.error_count > 5:
            self.integral_error["X"] = 0
            self.integral_error["Y"] = 0
            self.integral_error["Z"] = 0
            self.error_count = 0
        else:
            self.integral_error["X"] += x_error
            self.integral_error["Y"] += y_error
            self.integral_error["Z"] += z_error

        x_vel = (((x_error) * (kP))
                + ((x_derivative) * (kD))
                + ((x_integral) * (kI)))
        y_vel = (((y_error) * (kP))
                + ((y_derivative) * (kD))
                + ((y_integral) * (kI)))
        z_vel = -1 * (((z_error) * (kP))
                    + ((z_derivative) * (kD))
                    + ((z_integral) * (kI)))
        
        # x_vel = self.slew(vMax, jMax, slew, x_vel)
        x_vel = self.apply_velocity_constraints(x_vel)
        y_vel = self.apply_velocity_constraints(y_vel)
        z_vel = self.apply_velocity_constraints(z_vel, z_val=True)

        self.errors = {
            "X": x_error,
            "Y": y_error,
            "Z": z_error
        }

        return x_vel, y_vel, z_vel
    
    def slew(self, vMax, jMax, now, currentVel):
        inflectionTime = m.sqrt(vMax / jMax)
        totalTime = inflectionTime * 2 
        vInflection = vMax / 2
        aMax = (totalTime * jMax) / 2
        if(now < inflectionTime):
            velocity = (jMax * m.pow(now, 2)) / 2
            print(f"velocity : {velocity}")
        elif(now <= totalTime and now >= inflectionTime):
            velocity = vInflection + (aMax * (now - totalTime)) - ((jMax * pow((now - totalTime), 2)) / 2)
            print(f"velocity : {velocity}")
        else:
            velocity = currentVel
        return velocity

    def apply_velocity_constraints(self, speed, z_val=False):
        threshold = 19
        # These constraints need to be read from a settings file
        if not z_val and speed > threshold:
            speed = threshold
        elif z_val and speed < -threshold:
            speed = -threshold
        return speed

    def move_to_next_position(self, command: MovementCommand, startTime) -> bool:
        """
        Given a MovementCommand, send the appropriate AirSim API call
        to move the vehicle in the next direction.

        ## Inputs:
        - command [MovementCommand] data structure containing the pos,
                                    heading and speed of the next pos
                                    of the UAV.
                                    Members:
                                    - position [PosVec3]
                                    - heading [float]
                                    - speed [float]
        ## Outputs
        Boolean that signifies that the execution of the method was
        successful.
        """
        # TODO Add drivetrain once that is fixed
        if command.move_by == "position":
            state = self.airsim_client.getMultirotorState(
                vehicle_name=self.drone_id
            )
            position = position_to_list(
                state.kinematics_estimated.position
            )
            x_Vel, y_Vel, z_Vel = self.calculate_velocities(
                command.position,
                position,
                command, 
                startTime
            )
            if abs(z_Vel - self.last_velocities.vz) > 10 and self.last_command == "velocity":
                z_Vel = self.last_velocities.vz

            heading = command.heading
            
            # If we are step changing between velocities, cut it in half
            # if we exceed our threshold.
            if abs(x_Vel - self.last_velocities.vx) > 5:
                x_Vel = x_Vel / 2.0    
            if abs(y_Vel - self.last_velocities.vy) > 5:
                y_Vel = y_Vel / 2.0       
            if abs(z_Vel - self.last_velocities.vz) > 5:
                z_Vel = z_Vel / 2.0
            self.last_velocities.vx = x_Vel
            self.last_velocities.vy = y_Vel
            self.last_velocities.vz = z_Vel
            self.last_position.X = command.position.X
            self.last_position.Y = command.position.Y
            self.last_position.Z = command.position.Z
        elif command.move_by == "velocity":
            # TODO Find a way to incorporate the previous velocities
            x_Vel = self.last_velocities.vx + (command.velocity.vx * np.cos(np.radians(self.last_command.heading)))
            y_Vel = self.last_velocities.vy + (command.velocity.vx * np.sin(np.radians(self.last_command.heading)))
            z_Vel = self.interpolate_z_vel(command.velocity.vz) + self.last_velocities.vz
            print(f"last vel {self.last_velocities.vz}")
            print(f"oa vel {command.velocity.vz}")
            self.previous_velocities = {
                "VX": x_Vel,
                "VY": y_Vel,
                "VZ": z_Vel,
            }
            heading = self.last_command.heading
        self.log.info("{}|{}|velocities|{}".format(
                                datetime.utcnow(),
                                self.drone_id,
                                json.dumps([x_Vel, y_Vel, z_Vel])
                                )
                            )
        print(f'Z speed: {z_Vel}')
        self.airsim_client.moveByVelocityAsync(
                x_Vel,
                y_Vel,
                z_Vel,
                m.inf,
                yaw_mode=YawMode(False,heading),
                vehicle_name=self.drone_id
            )
        return True

    def interpolate_z_vel(self, next_z_vel):
        previous_z_vel = self.last_velocities.vz
        # We receive a new Z velocity that has some step
        # change difference in velocity, say (1.0 and 4.0)
        return previous_z_vel + next_z_vel