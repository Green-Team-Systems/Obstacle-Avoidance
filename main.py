# ===============================================================
# Created By: Tyler Fedrizzi
# Authors: Tyler Fedrizzi
# Created On: October 21st, 2021
# Updated On: October 21st, 2021
# 
# Description: Testing parameters for merging path planning and
#              obstacle avoidance together.
# ===============================================================
import json
import traceback

from matplotlib import use
import setup_path
import airsim
import time
import math as m
import numpy as np
import logging
import matplotlib.pyplot as plot



from mpl_toolkits import mplot3d
from multiprocessing import Queue, freeze_support

from obstacle_avoidance import ObstacleAvoidance
from path_planning import PathPlanning
from utils.data_classes import MovementCommand, PosVec3
from utils.distance_utils import ned_position_difference
from utils.position_utils import position_to_list


if __name__ == "__main__":

    freeze_support()

    FORMAT = '%(asctime)-15s %(message)s'
    use_oa = True
    logging.basicConfig(filename='logs/root.log',
                    level=logging.INFO,
                    format=FORMAT)
    drone_ids = ["Drone1", "Drone2", "Drone3"]
    lidar_ids = ["LidarSensor1", "LidarSensor2", "LidarSensor3"]
    path_planning_queues = [Queue(), Queue(), Queue()]
    planners = list()
    avoiders = list()
    dist_threshold = 2.0 # meters
    positions = [list(), list(), list()]

    airsim_client = airsim.MultirotorClient()
    airsim_client.confirmConnection()

    with open("blocksTrajectory.json", "r") as f:
        trajectory = json.loads(f.read())
    if use_oa:
        for drone_id, queue, sensor_name in zip(drone_ids, path_planning_queues, lidar_ids):
            oa_module = ObstacleAvoidance(
                queue,
                drone_id,
                sensor_name,
                simulation=True
            )
            avoiders.append(oa_module)
    for drone_id, queue in zip(drone_ids, path_planning_queues):
        path_plan_module = PathPlanning(
            queue,
            drone_id,
            simulation=True
        )
        planners.append(path_plan_module)

    for planner in planners:
        planner.start()
    if use_oa:
        for avoider in avoiders:
            avoider.start()

    # The drone starts in the 0,0,0 coordinate position
    current_position = PosVec3()
    count = 0
    try:
        for point in trajectory["Trajectory"]:
            # Generate the next movement command to the system, giving
            # this a priority of 2 (user priority, high-level goal)
            heading = np.arctan2(point["Y"],point["X"])
            heading = m.degrees(heading)
            command = MovementCommand(
                position=PosVec3(
                    X=point["X"],
                    Y=point["Y"],
                    Z=point["Z"],
                ),
                heading=heading,
                priority=2
            )
            for i, queue in enumerate(path_planning_queues):
                if i == 1:
                    command = MovementCommand(
                        position=PosVec3(
                            X=point["X"] - 2,
                            Y=point["Y"] - 2,
                            Z=point["Z"],
                        ),
                        heading=heading,
                        priority=2
                    )
                elif i == 2:
                    command = MovementCommand(
                        position=PosVec3(
                            X=point["X"] - 2,
                            Y=point["Y"] + 2,
                            Z=point["Z"],
                        ),
                        heading=heading,
                        priority=2
                    )
                queue.put(command)
            arrived = False
            while not arrived:
                state = airsim_client.getMultirotorState(vehicle_name="Drone1")
                position = position_to_list(state.kinematics_estimated.position)
                positions[0].append(position.X)
                positions[1].append(position.Y)
                positions[2].append(-1* position.Z)
                command = MovementCommand(
                    position=PosVec3(
                        X=point["X"],
                        Y=point["Y"],
                        Z=point["Z"],
                    ),
                    heading=heading,
                    priority=2
                )
                dist_to_target = ned_position_difference(
                    PosVec3(
                        X=point["X"],
                        Y=point["Y"],
                        Z=point["Z"],
                    ),
                    position
                )
                if count % 4 == 0:
                    #print(f"Current Position: {position} Target Position: {command.position}")
                    0
                if dist_to_target < dist_threshold:
                    #print(dist_to_target)
                    arrived = True
                else:
                    if len(path_planning_queues) > 1:
                        for i, queue in enumerate(path_planning_queues):
                            if i == 1:
                                command = MovementCommand(
                                    position=PosVec3(
                                        X=point["X"] - 2,
                                        Y=point["Y"] - 2,
                                        Z=point["Z"],
                                    ),
                                    heading=heading,
                                    priority=2
                                )
                            elif i == 2:
                                command = MovementCommand(
                                    position=PosVec3(
                                        X=point["X"] - 2,
                                        Y=point["Y"] + 2,
                                        Z=point["Z"],
                                    ),
                                    heading=heading,
                                    priority=2
                                )
                            queue.put(command)
                count += 1
                time.sleep(0.25)
    except Exception:
        traceback.print_exc()
    finally:
        print("Trajectory completed!")
        airsim_client.reset()
        if use_oa:
            for avoider in avoiders:
                avoider.terminate()
        for planner in planners:
            planner.terminate()

        print("Test complete. Check the logs!")

        ax = plot.axes(projection='3d')
        ax.scatter3D(positions[0], positions[1], positions[2])

        plot.show()
