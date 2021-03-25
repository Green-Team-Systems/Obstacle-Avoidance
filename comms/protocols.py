"""
def propagate_coordinates(client, comm_matrix: np.array, drones: np.array):
    for i, drone_comm_params in enumerate(comm_matrix):
        for j, individual_param in enumerate(drone_comm_params):
            if comm_matrix[i, j] == True and len(drones[find_name(i)].swarm_positions) < len(drones):
                # packet build_comms_packet()
                # drone_i.communicate(drone_j, packet, fade=0.2, max_dist=dist, packet_drop="Gaussian")
                drones[find_name(i)].swarm_positions.append(drones[find_name(j)].pos_vec3)
"""

"""
def update_communication_matrix(client, comm_matrix: np.array, drones: np.array) -> bool:
    for i, parameter_list in enumerate(comm_matrix):
        for j, individual_list in enumerate(parameter_list):
            if i != j:
                # i will give you the current drone, e.g. "A"
                # j will give you the comparison drone, e.g. "B, C, ..."
                comparison_drone = drones[find_name(j)].gps_pos_vec3
                # Get comms data for the drone we want "A" in relation to the comparison drone "B, C, ..."
                comm_matrix[i, j] = client.getCommunicationsData(
                    comparison_drone[0], # latitude
                    comparison_drone[1], # longitude
                    comparison_drone[2], # altitude
                    vehicle_name=find_name(i)).can_communicate
            else:
                comm_matrix[i, j] = True
"""