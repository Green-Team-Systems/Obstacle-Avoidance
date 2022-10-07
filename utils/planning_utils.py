# =============================================================================
# Copyright 2022. Codex Laboratories LLC
#
# Created By: Tyler Fedrizzi
# Created On: 6/4/2022
#
# Description: Planning Utilities for the SWARM platform
# =============================================================================
import logging
import time

from cmath import cos, sin
from datetime import datetime
from numpy import arctan2, degrees

from utils.data_classes import Orientation, PosVec3, TrajectoryType
from utils.distance_utils import ned_position_difference, ned_position_difference_x_y_only
from utils.position_utils import PRECISION

class Planner():
    """
    A planning class that is utilized to generate trajectories for
    agents to move from point A to point B.

    ## Inputs:
    - initial_position [PosVec3] The initialized position of the agent
                                 in meters.

    """

    def __init__(self,
                 initial_position: PosVec3,
                 initial_orientation: Orientation,
                 log: logging.Logger,
                 drone_id: str,
                 completed_radius: float = 2.0,
                 velocity_max: float = 3.0,
                 velocity_min: float = 1.0,
                 acceleration_max: float = 2.0,
                 traj_type: TrajectoryType = TrajectoryType.LINE) -> None:
        self.initial_position = initial_position  # Meters
        self.current_position = initial_position  # Meters
        self.orientation = initial_orientation  # Radians
        self.current_goal = PosVec3()  # Meters
        self.previous_goal = PosVec3()  # Meters
        # What is the distance that is considered to have "arrived" at
        # the current goal.
        self.completed_radius = completed_radius  # Meters
        self.velocity_max = velocity_max  # Meters / sec
        self.velocity_min = velocity_min  # Meters / sec
        self.acceleration_max = acceleration_max  # Meters / sec / sec
        # Maximum distance to travel
        self.maximum_distance = 0.0  # Meters
        self.traj_type = traj_type
        self.trajectory = list()
        self.max_process_time = 0.02  # seconds (20 milliseconds)
        self.log = log
        self.drone_id = drone_id

    def build_trajectory(self) -> bool:
        """
        Given a starting position and a goal position, build a
        trajectory from that point to the end point. Before this method
        is called, the current goal is updated

        ## Inputs:
        - None

        ## Outputs:
        - A boolean value determining if a trajectory could be
          successfully created.
        """
        dist_to_goal = ned_position_difference(self.current_position,
                                               self.current_goal)
        self.maximum_distance = dist_to_goal
        # If the goal is further then the maximum velocity of the system
        # then generate a path to get there
        if dist_to_goal > self.velocity_max:
            success = self.build_path_segements(dist_to_goal)
            if not success:
                self.log.info("{}|{}|planning|{}".format(
                    datetime.utcnow(),
                    self.drone_id,
                    "Planning failed! No plan generated!"
                )
                )
                return False
        else:
            self.trajectory = [
                dict(pos=self.current_goal,
                    time=1.5,
                    heading=round(
                        degrees(
                            arctan2(
                                self.current_goal.Y,
                                self.current_goal.X)),
                            PRECISION),
                    velocity=self.velocity_max)
                ]

        if len(self.trajectory) == 0:
            self.log.info("{}|{}|planning|{}".format(
                    datetime.utcnow(),
                    self.drone_id,
                    "Planning failed! No trajectory generated!"
                )
                )
            return False
        else:
            return True

    def build_path_segements(self,
                             dist_to_goal: float) -> list:
        """
        Given a goal and your current position, move to said goal along 
        a set of path segments, which are generated based upon the
        algorithm selected.
        """
        # Simplest Case -> build a set of straight lines that meets the
        # defined constraints of the system
        build_time = time.time()
        if self.traj_type == TrajectoryType.LINE:
            start = time.time()
            # Determine how many points will be on the line
            numb_segments = 1
            seg_length = dist_to_goal / numb_segments
            while seg_length > self.velocity_max and (time.time() - start < self.max_process_time):
                seg_length = dist_to_goal / numb_segments
                numb_segments += 1
            velocity = round(dist_to_goal / numb_segments, PRECISION)
            self.trajectory = list()
            heading = round(arctan2(self.current_goal.Y, self.current_goal.X), PRECISION)
            last_point = self.current_position

            print("Starting point for trajectory: {}".format(last_point))

            for i in range(1, numb_segments + 1):
                if (i == numb_segments):
                    Y=round(self.current_goal.Y, PRECISION)
                    X=round(self.current_goal.X, PRECISION)
                else:
                    Y=round(last_point.Y + (velocity *
                        sin(heading).real), PRECISION)
                    X=round(last_point.X + (velocity *
                        cos(heading).real), PRECISION)
                last_point = PosVec3(
                    X=X,
                    Y=Y,
                    Z=self.current_goal.Z
                )
                self.trajectory.append(dict(
                    pos=last_point,
                    time=round(seg_length / velocity, PRECISION),
                    heading=round(degrees(heading), PRECISION),
                    velocity=velocity))
            goal_error = ned_position_difference_x_y_only(
                             self.trajectory[-1]["pos"],
                             self.current_goal)
            constraints_satisified = self.check_trajectory_validity()
            if constraints_satisified:
                self.log.info("{}|{}|planning|{}".format(
                    datetime.utcnow(),
                    self.drone_id,
                    "Plan accepted! Trajectory: {}".format(
                        self.trajectory
                    )
                )
                )
                self.log.info("{}|{}|planning|{}".format(
                    datetime.utcnow(),
                    self.drone_id,
                    "Time to plan: {}".format(
                        time.time() - build_time
                    )
                )
                )
                self.log.info("{}|{}|planning|{}".format(
                    datetime.utcnow(),
                    self.drone_id,
                    "Goal Error: {}".format(
                        goal_error
                    )
                )
                )
                return True
            else:
                return False

    def check_trajectory_validity(self) -> bool:
        """
        Given a trajectory (list of points), verify that the set of all
        points is valid within the set of constraints set by the user.

        ## Inputs:
        - None

        ## Outputs:
        - boolean value representing the validity of the trajectory
        """
        if len(self.trajectory) > 2:
            for i, pos_vec in enumerate(self.trajectory):
                if i > 0:
                    # Assuming we don't command velocities and let the
                    # positional PID handle that.
                    vel_diff = (pos_vec["velocity"]
                                - self.trajectory[i - 1]["velocity"])
                    accel = vel_diff / pos_vec["time"]
                    if accel > self.acceleration_max:
                        self.log.info("{}|{}|planning|{}".format(
                            datetime.utcnow(),
                            self.drone_id,
                            "Error! Planning failed!"
                            "Maximum acceleration constraint surpassed!"
                        )
                        )
                        return False
            pos_diff = ned_position_difference_x_y_only(
                           self.trajectory[-1]["pos"],
                           self.current_goal)
            if pos_diff > self.completed_radius:
                self.log.info("{}|{}|planning|{}".format(
                            datetime.utcnow(),
                            self.drone_id,
                            "Planning failed!"
                            "Desired path does not terminate at correct point!"
                            "Error: {} Meters".format(pos_diff)
                        )
                        )
                self.log.info("{}|{}|planning|{}".format(
                            datetime.utcnow(),
                            self.drone_id,
                            "Failed Trajectory is {}".format(self.trajectory)
                        )
                        )
                return False
            return True
        else:
            return True

    def check_trajectory_progress(self):
        """
        Monitor progress to each waypoint and remove that waypoint from
        the front of the list.

        # Inputs:
        - None

        # Outputs:
        - None
        """
        pos_diff = ned_position_difference_x_y_only(
            self.current_position, self.trajectory[0]["pos"]
        )

        if pos_diff <= self.completed_radius:
            self.log.info("{}|{}|planning|{}".format(
                            datetime.utcnow(),
                            self.drone_id,
                            "Point Completed: {}".format(self.trajectory[0])
                        )
                    )
            self.trajectory.pop(0)
