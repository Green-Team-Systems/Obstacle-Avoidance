import time
from db.data_methods import save_initial_drone_position
import copy


def enable_control(client, drones: dict) -> None:
    for vehicle_name in drones:
        client.enableApiControl(True, vehicle_name)
        client.armDisarm(True, vehicle_name)


def disable_control(client, drones: dict) -> None:
    for vehicle_name in drones:
        client.armDisarm(False, vehicle_name)
        client.enableApiControl(False, vehicle_name)


def takeoff(client, drones: list) -> None:
    """
       Make all vehicles takeoff, one at a time and return the
       pointer for the last vehicle takeoff to ensure we wait for
       all drones
    """
    vehicle_pointers = []
    for drone_name in list(drones):
        vehicle_pointers.append(client.takeoffAsync(vehicle_name=drone_name))
    # All of this happens asynchronously. Hold the program until the last vehicle
    # finishes taking off.
    return vehicle_pointers[-1]


def update_all_drone_positions(client, drones: list):
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


def position_to_list(position_vector) -> list:
    return [position_vector.x_val, position_vector.y_val, position_vector.z_val]


def gps_position_to_list(gps_vector) -> list:
    return [gps_vector.latitude, gps_vector.longitude, gps_vector.altitude]


def gps_velocity_to_list(velocity_vector) -> list:
    return [velocity_vector.x_val, velocity_vector.y_val, velocity_vector.z_val]


def fly_to_new_positions(client, drones: dict) -> None:
    for drone_name in drones:
        z_coord = ensure_negative_z_coordinates(drones[drone_name].pos_vec3[2])
        client.moveToPositionAsync(
            drones[drone_name].pos_vec3[0],
            drones[drone_name].pos_vec3[1],
            z_coord,
            drones[drone_name].forward_speed,
            vehicle_name=drone_name)
        time.sleep(0.1)


def move_drone_by_velocity(client, drones: dict):
    for drone_name in drones:
        pass


def ensure_negative_z_coordinates(z_value):
    if z_value > 0:
        return z_value * -1
    else:
        return z_value


def transform_to_standard_basis_coordinates(new_position: list, vehicle_offsets: list) -> list:
    # Each drone reports its position in the x,y,z plane relative to it's own starting position,
    # which is relative to the PlayerStart position in Unreal Engine. You must compensate the reported
    # position of each drone with it's starting position relative to the standard basis to properly calculate
    # the average position for each drone.
    # the average position for each drone.
    pos = new_position
    new_position[0] += vehicle_offsets[0]
    new_position[1] += vehicle_offsets[1]
    new_position = pos
    # new_position[2] += vehicle_offsets[2]


def transform_to_relative_basis_coordinates(new_position: list, vehicle_offsets: list) -> list:
    # Transform the coordinates of each drone their independent representation frames.
    pos = new_position
    pos[0] -= vehicle_offsets[0]
    pos[1] -= vehicle_offsets[1]
    new_position = pos
    # new_position[2] -= vehicle_offsets[2]

def set_initial_positions_of_all_drones(scenario, settings_info, database, sim_id):
    for swarm in scenario.swarms:
        for drone_name in swarm.drones:
            positions = settings_info['Vehicles'][drone_name]
            drone = swarm.drones[drone_name]
            drone.starting_pos_vec3[0] = positions['X']
            drone.pos_vec3[0] = positions['X']
            drone.real_pos_vec3[0] = positions['X']
            drone.starting_pos_vec3[1] = positions['Y']
            drone.pos_vec3[1] = positions['Y']
            drone.real_pos_vec3[1]= positions['Y']
            drone.starting_pos_vec3[2] = positions['Z']
            drone.pos_vec3[2] = positions['Z']
            drone.real_pos_vec3[2] = positions['Z']
            swarm.positions[drone_name] = drone.pos_vec3
            swarm.real_positions[drone_name] = drone.real_pos_vec3

def print_drone_positions(swarm):
    for drone_name in swarm.drones:
        pos = swarm.drones[drone_name].pos_vec3
        print('\n Name: {}\t Position: X:{} Y:{} Z:{}'.format(drone_name, pos[0], pos[1], pos[2]) )