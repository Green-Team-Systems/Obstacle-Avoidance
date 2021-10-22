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
import setup_path
import airsim
import time

from multiprocessing import Queue

from obstacle_avoidance import ObstacleAvoidance
from path_planning import PathPlanning
from utils.data_classes import MovementCommand, PosVec3
from utils.distance_utils import ned_position_difference
from utils.position_utils import position_to_list


if __name__ == "__main__":
    drone_id = "Drone1"
    path_planning_queue = Queue()
    dist_threshold = 3.0 # meters

    airsim_client = airsim.MultirotorClient()
    airsim_client.confirmConnection()

    with open("trajectory.json", "r") as f:
        trajectory = json.loads(f.read())

    oa_module = ObstacleAvoidance(
        path_planning_queue,
        drone_id,
        simulation=True
    )
    path_plan_module = PathPlanning(
        path_planning_queue,
        drone_id,
        simulation=True
    )

    oa_module.start()
    path_plan_module.start()

    # The drone starts in the 0,0,0 coordinate position
    current_position = PosVec3()

    for point in trajectory["trajectory"]:
        # Generate the next movement command to the system, giving
        # this a priority of 2 (user priority, high-level goal)
        command = MovementCommand(
            position=PosVec3(
                X=point["X"],
                Y=point["Y"],
                Z=point["Z"],
            ),
            heading=point["heading"],
            priority=2
        )
        path_planning_queue.put(command)
        arrived = False
        while not arrived:
            state = airsim_client.getMultirotorState(vehicle_name=drone_id)
            position = position_to_list(state.kinematics_estimated.position)
            dist_to_target = ned_position_difference(
                command.position,
                position
            )
            if dist_to_target < dist_threshold:
                arrived = True
            time.sleep(0.1)

    print("Trajectory completed!")
    oa_module.terminate()
    path_plan_module.terminate()

    print("Test complete. Check the logs!")