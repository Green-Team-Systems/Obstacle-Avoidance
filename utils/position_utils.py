# ===============================================================
# Copyright 2021. Codex Laboratories LLC
# Created By: Tyler Fedrizzi
# Authors: Tyler Fedrizzi
# Created On: September 6th, 2020
# Updated On: August 11th, 2021
# 
# Description: Methods to control the position of the drones
# ===============================================================
import time

from utils.data_classes import PosVec3


def enable_control(client, drones: list) -> None:
    """
    Enable API control of all drones and then arm
    each drone.

    Inputs:
    - client [AirSim] - AirSim client to make requests
    - drones [list] - list of drone names
    """
    for vehicle_name in drones:
        client.enableApiControl(True, vehicle_name)
        client.armDisarm(True, vehicle_name)


def disable_control(client, drones: list) -> None:
    """
    Disable API control and disarm the drones.

    Inputs:
    - client [AirSim] - AirSim client to make requests
    - drones [list] - list of drone names
    """
    for vehicle_name in drones:
        client.armDisarm(False, vehicle_name)
        client.enableApiControl(False, vehicle_name)


def takeoff(client, drones: list) -> bool:
    """
    Make all vehicles takeoff, one at a time and return the
    pointer for the last vehicle takeoff to ensure we wait for
    all drones.

    Inputs:
    - client [AirSim] - AirSim client to make requests
    - drones [list] - list of drone names
    """
    for drone_name in list(drones):
        print(drone_name)
        client.takeoffAsync(vehicle_name=drone_name)
        time.sleep(0.01)
    # All of this happens asynchronously. Hold the program until the last
    # vehicle finishes taking off.
    return True


def update_all_drone_positions(client, drones: list):
    """
    Update the position of all drones by performing a call to AirSim
    and return a MultirotorState object containing all of the info
    about the drone.

    Inputs:
    - client [AirSim] - AirSim client to make requests
    - drones [list] - list of drone names
    """
    for drone_name in drones:
        state_data = client.getMultirotorState(vehicle_name=drone_name)
        drones[drone_name].pos_vec3 = position_to_list(
            state_data.kinematics_estimated.position)
        drones[drone_name].real_pos_vec3 = position_to_list(
            state_data.kinematics_estimated.position)
        drones[drone_name].gps_pos_vec3 = gps_position_to_list(
            state_data.gps_location)
        drones[drone_name].current_velocity = gps_velocity_to_list(
            state_data.kinematics_estimated.linear_velocity)


def position_to_list(position_vector,
                     starting_position: PosVec3 = PosVec3(),
                     frame="local") -> PosVec3:
    """
    Given a vector from AirSim, generate a List to iterate
    through.

    Inputs:
    - position_vector [AirSim Vector3] the x,y,z position vector

    Outputs:
    List of x,y,z position
    """
    if frame == "local":
        return PosVec3(X=position_vector.x_val,
                       Y=position_vector.y_val,
                       Z=position_vector.z_val,
                       frame=frame)
    else:
        position = PosVec3(X=position_vector.x_val,
                           Y=position_vector.y_val,
                           Z=position_vector.z_val,
                           frame='local')
        transform_to_global_coordinate_frame(position, starting_position)
        return position


def gps_position_to_list(gps_vector) -> list:
    """
    Given a vector from AirSim, generate a List to iterate
    through.

    Inputs:
    - gps_vector [AirSim] the lat,lon,altitude position vector

    Outputs:
    List of x,y,z position
    """
    return [gps_vector.latitude,
            gps_vector.longitude,
            gps_vector.altitude]


def gps_velocity_to_list(velocity_vector) -> list:
    """
    Given a vector from AirSim, generate a List to iterate
    through.

    Inputs:
    - velocity_vector [AirSim] the x,y,z velocity vector

    Outputs:
    List of x,y,z velocity
    """
    return [velocity_vector.x_val,
            velocity_vector.y_val,
            velocity_vector.z_val]


def fly_to_new_positions(client, drones: dict) -> None:
    """
    Given the list of drones, move the drone to the new position
    that has been given by the user.

    Inputs;
    - client [AirSim] client to communicate with AirSim.
    - drones [dict] a data object containing the drones, referenced by
                    the drone ID. 
    """
    for drone_name in drones:
        z_coord = ensure_negative_z_coordinates(drones[drone_name].pos_vec3[2])
        client.moveToPositionAsync(
            drones[drone_name].pos_vec3[0],
            drones[drone_name].pos_vec3[1],
            z_coord,
            drones[drone_name].forward_speed,
            vehicle_name=drone_name)
        time.sleep(0.01) # required for concurrency reasons


def ensure_negative_z_coordinates(z_value):
    """
    # TODO Remove this and switch to the absolute function
    """
    if z_value > 0:
        return z_value * -1
    else:
        return z_value


def transform_to_global_coordinate_frame(position: PosVec3,
                                         starting_position: PosVec3) -> None:
    """
    Each drone reports its position in the x,y,z plane relative to it's
    own starting position, which is relative to the PlayerStart position
    in Unreal Engine. You must compensate the reported position of each
    drone with it's starting position relative to the standard basis to
    properly calculate the average position for each drone.

    Inputs:
    - position [list] The new position vector for the drone.
    - starting_position [list] Initial position of the drone at
                             initialization
    
    Outputs:
    An updated position vector.
    """
    position.X += starting_position.X
    position.Y += starting_position.Y
    position.Z += starting_position.Z
    position.frame = "global"


def transform_to_standard_basis_coordinates(current_pos: list,
                                            start_pos: list) -> None:
    current_pos[0] += start_pos[0]
    current_pos[1] += start_pos[1]
    current_pos[2] -= start_pos[2]



def transform_to_local_coordinate_frame(position: PosVec3,
                                        starting_position: PosVec3) -> None:
    """
    Transform the coordinates of each drone from the global reference
    frame to the local reference frame of each drone, which is
    based upon the origin of each drone.

    Inputs:
    - new_position [list] The new position vector for the drone.
    - vehicle_offsets [list] Initial position of the drone at
                             initialization
    
    Outputs:
    An updated position vector.
    """
    position.X -= starting_position.X
    position.Y -= starting_position.Y
    position.Z += starting_position.Z
    position.frame = "local"


def transform_to_relative_basis_coordinates(current_pos: list,
                                            start_pos: list) -> None:
    current_pos[0] -= start_pos[0]
    current_pos[1] -= start_pos[1]
    current_pos[2] += start_pos[2]


# TODO This should be a main process function that just prints out
# the current location of the drones and the mission sets. Shouldn't
# be referenced in this way.
def print_drone_positions(swarm):
    """
    Print the position of each drone in the swarm.

    Inputs:
    - swarm [Swarm] the swarm that holds the reference to each drone.
    """
    for drone_name in swarm.drones:
        pos = swarm.drones[drone_name].pos_vec3
        print('\n Name: {}\t Position: X:{} Y:{} Z:{}'.format(drone_name,
                                                              pos[0],
                                                              pos[1],
                                                              pos[2]) )

def update_drone_position(client, drone: dict):
    """
    Update a single drone's position.

    Inputs:
    - client [AirSim] AirSim client to communicate with the sim
    - drone [dict] dictionary of information for the drone.
    """
    state_data = client.getMultirotorState(vehicle_name=drone["drone_name"])
    drone["pos_vec3"] = position_to_list(
        state_data.kinematics_estimated.position)

def update_drone_position(client, drone: dict):
    """
    Update a single drone's position.

    Inputs:
    - client [AirSim] AirSim client to communicate with the sim
    - drone [dict] dictionary of information for the drone.
    """
    state_data = client.getMultirotorState(vehicle_name=drone["drone_name"])
    position = state_data.kinematics_estimated.position
    drone["pos_vec3"] = [position.x_val,
                         position.y_val,
                         position.z_val]


def update_all_drone_positions_for_camera(client, drones: dict):
    """
    Update the position struct for all drones in the swarm, so that
    the recording drone can average their positions and move to that
    direction.

    Inputs:
    - client [AirSim] AirSim client to communicate with the sim
    - drone [dict] dictionary of information for the drone.
    """
    for drone_name in drones.keys():
        state_data = client.getMultirotorState(vehicle_name=drone_name)
        position = state_data.kinematics_estimated.position
        drones[drone_name]["pos_vec3"] = [position.x_val,
                                          position.y_val,
                                          position.z_val]

def average_drone_positions_for_camera(camera_position: dict,
                                       swarm_positions: dict) -> list:
    x = 0.0
    y = 0.0
    z = 0.0
    numb_drones = 0.0

    for vehicle_name, info in swarm_positions.items():
        # Position is a list like [X,Y,Z]
        position = info["pos_vec3"]
        x += position[0]
        y += position[1]
        z += position[2]
        numb_drones += 1.0

    x = x / numb_drones
    y = y / numb_drones
    z = -50

    return [x,y,z]
