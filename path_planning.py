# =============================================================================
# Created By: Tyler Fedrizzi
# Authors: Tyler Fedrizzi
# Created On: September 6th, 2020
# Last Modified: Septebmer 5th, 2021
#
# Description: Module for implementing movement commands to the Aircraft
# =============================================================================
import traceback
import logging
import time
import copy
import numpy as np
import math as m
import json

from multiprocessing import Process
from queue import Queue
from datetime import datetime
from multiprocessing.queues import Empty

from utils.data_classes import Orientation, PosVec3, MovementCommand, VelVec3
from utils.killer_utils import GracefulKiller
from utils.planning_utils import Planner
from utils.airsim_utils import PoseAirSimConnector
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

    def __init__(self,
                 queue: Queue,
                 drone_id: str,
                 user_options: dict = None,
                 simulation: bool = False,
                 use_airsim: bool = True) -> None:
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
        if use_airsim:
            self.airsim_connector = None
            self.build_airsim_client = True
        # TODO Add status updates
        self.status = None
        self.last_command = None
        self.heading = float()
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
        self.planner = None

    def start(self):
        """
        Override Process start function and update status
        """
        Process.start(self)
        # TODO Update status of operations

    def is_new_command(self, command: MovementCommand) -> bool:
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
        FORMAT = '%(asctime)s %(message)s'
        logging.basicConfig(format=FORMAT,
                            level=logging.INFO)
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
        killer = GracefulKiller()

        if self.simulation and self.build_airsim_client:
            self.airsim_connector = PoseAirSimConnector(
                starting_position=PosVec3(),
                starting_orientation=Orientation(),
                log=self.log,
                drone_id=self.drone_id
            )
            self.airsim_connector.build_airsim_client()
            # Wait 1/2 second to get the system started
            time.sleep(0.5)
        self.planner = Planner(
            self.airsim_connector.current_position,
            self.airsim_connector.current_orientation,
            self.log,
            self.drone_id
        )
        if not self.airborne and self.simulation and self.build_airsim_client:
            self.airsim_connector.takeoff()
        for _ in range(2):
            self.command_queue.put("Takeoff Completed")
            time.sleep(0.001)
        # TODO What if there is a collision or error?

        # Main Execution
        self.log.info("{}|{}|message|{}".format(
            datetime.utcnow(),
            self.drone_id,
            "Beginning main path planning execution"
        )
        )
        startTime = datetime.utcnow()
        # self.command_queue.join()
        while not killer.kill_now:
            # TODO This should be a thread that is used to grab the updated
            # position from the GPS system or localization systems
            self.airsim_connector.update_pose()
            self.planner.current_position = self.airsim_connector.global_position
            self.planner.orientation = self.airsim_connector.current_orientation
            self.airsim_connector.propagate_pose()
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
                    try:
                        assert isinstance(command, MovementCommand)

                        if (self.is_new_command(command) == True and command.move_by == "position"):
                            self.log.info("{}|{}|commanded_position|{}".format(
                                datetime.utcnow(),
                                self.drone_id,
                                command)
                            )
                            self.last_command = copy.deepcopy(command)
                            self.planner.current_goal = command.position
                            self.last_command = command
                            self.planner.current_position = self.airsim_connector.current_position
                            self.planner.build_trajectory()
                            if len(self.planner.trajectory) > 0:
                                position = PosVec3(
                                    X=self.planner.trajectory[0]["pos"].X,
                                    Y=self.planner.trajectory[0]["pos"].Y,
                                    Z=self.planner.trajectory[0]["pos"].Z)
                                new_command = MovementCommand(position=position,
                                                              heading=self.planner.trajectory[0]["heading"],
                                                              move_by="position")
                                self.move_to_next_position(
                                    new_command, startTime)
                        else:
                            position = PosVec3(
                                X=self.planner.trajectory[0]["pos"].X,
                                Y=self.planner.trajectory[0]["pos"].Y,
                                Z=self.planner.trajectory[0]["pos"].Z)
                            new_command = MovementCommand(position=position,
                                                          heading=self.planner.trajectory[0]["heading"],
                                                          move_by="position")
                            self.move_to_next_position(
                                new_command, startTime)
                        print("length: ")
                        print(self.planner.trajectory)
                        if len(self.planner.trajectory) > 0:
                            self.planner.check_trajectory_progress()

                    except Exception:
                        traceback.print_exc()
                # self.commands.clear()
                except Empty:
                    try:
                        if len(self.planner.trajectory) > 0:
                            position = self.planner.trajectory[0]["pos"]
                            new_command = MovementCommand(position=position,
                                                          heading=self.planner.trajectory[0]["heading"],
                                                          move_by="position")
                            self.move_to_next_position(new_command, startTime)
                        time.sleep(0.02)
                    except Exception:
                        pass
            else:
                time.sleep(0.1)
        self.log.info("{}|{}|message|{}".format(
            datetime.utcnow(),
            self.drone_id,
            "Shutdown initiated!"
        )
        )

        # Clean shutdown by disarming and disabling API control
        if self.simulation and self.build_airsim_client:
            self.airsim_connector.shutdown()
            time.sleep(0.5)

        self.log.info("{}|{}|message|{}".format(
            datetime.utcnow(),
            self.drone_id,
            "Shutdown completed!"
        )
        )

    def calculate_velocities(self, target_pos, startTime):
        kP = 0.40
        kI = 0.00
        kD = 0.05
        vMax = 20
        jMax = 25
        # print(target_pos.Z, current_pos.Z)
        x_error = target_pos.X - self.airsim_connector.global_position.X
        y_error = target_pos.Y - self.airsim_connector.global_position.Y
        z_error = -1 * (target_pos.Z - self.airsim_connector.global_position.Z)

        # if(self.check_for_new_command(command) == True):
        #     startTime = datetime.utcnow()
        #     print(startTime)
        now = datetime.utcnow()
        slew = (now - startTime).total_seconds()
        dt = (now - self.previous_time).total_seconds()
        self.previous_time = now
        if dt == 0:
            dt = 0.02

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
            velocity = vInflection + \
                (aMax * (now - totalTime)) - \
                ((jMax * pow((now - totalTime), 2)) / 2)
            print(f"velocity : {velocity}")
        else:
            velocity = currentVel
        return velocity

    def apply_velocity_constraints(self, speed, z_val=False):
        threshold = 15
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
            x_Vel, y_Vel, z_Vel = self.calculate_velocities(
                command.position,
                startTime
            )
            if abs(z_Vel - self.last_velocities.vz) > 10 and self.last_command == "velocity":
                z_Vel = self.last_velocities.vz

            heading = command.heading

            # If we are step changing between velocities, cut it in half
            # if we exceed our threshold.
            if abs(x_Vel - self.last_velocities.vx) > 10:
                x_Vel = x_Vel / 2.0
            elif abs(x_Vel - self.last_velocities.vx) > 5:
                x_Vel = x_Vel / 1.5
            if abs(y_Vel - self.last_velocities.vy) > 10:
                y_Vel = y_Vel / 2.0
            elif abs(y_Vel - self.last_velocities.vy) > 5:
                y_Vel = y_Vel / 1.5
            if abs(z_Vel - self.last_velocities.vz) > 10:
                z_Vel = z_Vel / 2.0
            elif abs(z_Vel - self.last_velocities.vz) > 5:
                z_Vel = z_Vel / 1.5
            self.last_velocities.vx = x_Vel
            self.last_velocities.vy = y_Vel
            self.last_velocities.vz = z_Vel
            self.last_position.X = command.position.X
            self.last_position.Y = command.position.Y
            self.last_position.Z = command.position.Z
            # heading = self.last_command.heading
            """
            
            self.log.info("{}|{}|velocities|{}".format(# All agents start from their
                                    datetime.utcnow(),
                                    self.drone_id,
                                    json.dumps([x_Vel, y_Vel, z_Vel])
                                    )
                                )
            """
            self.airsim_connector.velocity_command(xVel=x_Vel,
                                                   yVel=y_Vel,
                                                   zVel=z_Vel,
                                                   speed=5,
                                                   heading=heading)

        elif command.move_by == "velocity":
            # TODO Find a way to incorporate the previous velocities
            x_Vel = self.last_velocities.vx + \
                (command.velocity.vx * np.cos(np.radians(self.heading)))
            y_Vel = self.last_velocities.vy + \
                (command.velocity.vx * np.sin(np.radians(self.heading)))
            z_Vel = self.interpolate_z_vel(command.velocity.vz)
            self.previous_velocities = {
                "VX": x_Vel,
                "VY": y_Vel,
                "VZ": z_Vel,
            }

            self.airsim_connector.velocity_command(xVel=x_Vel,
                                                   yVel=y_Vel,
                                                   zVel=z_Vel,
                                                   speed=5,
                                                   heading=heading)
        elif command.move_by == "acceleration":
            self.log.info("{}|{}|acceleration|{}".format(
                datetime.utcnow(),
                self.drone_id,
                json.dumps(
                    [float(command.acceleration.roll),
                     float(command.acceleration.pitch),
                     float(command.acceleration.yaw),
                     float(command.acceleration.throttle)])
            )
            )
            if command.acceleration.throttle > 1.0:
                command.acceleration.throttle = 1.0
            elif command.acceleration.throttle < 0.0:
                command.acceleration.throttle = 0.1

            self.airsim_connector.acceleration_command(
                roll=float(command.acceleration.roll),
                pitch=float(command.acceleration.pitch),
                yaw=float(command.acceleration.yaw),
                throttle=float(command.acceleration.throttle),
                heading=heading
            )
        elif command.move_by == "yaw":
            self.airsim_connector.yaw_command(heading=command.heading,
                                              timeout=5.0,
                                              margin=1.0)
        return True

    def interpolate_z_vel(self, next_z_vel):
        previous_z_vel = self.last_velocities.vz
        # We receive a new Z velocity that has some step
        # change difference in velocity, say (1.0 and 4.0)
        return previous_z_vel + next_z_vel