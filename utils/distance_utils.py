# ================================================================
# Created by: Tyler Fedrizzi
# Created On: 21 March 2020
#
# Description: Utilty methods for calculating distance
# ================================================================
import numpy as np
import math
from utils.name_utils import find_name


def haversine(lat1, lon1, lat2, lon2):
    # distance between latitudes 
    # and longitudes 
    dLat = (lat2 - lat1) * math.pi / 180.0
    dLon = (lon2 - lon1) * math.pi / 180.0
  
    # convert to radians 
    lat1 = (lat1) * math.pi / 180.0
    lat2 = (lat2) * math.pi / 180.0
  
    # apply formulae 
    a = (pow(math.sin(dLat / 2), 2) + 
         pow(math.sin(dLon / 2), 2) * 
             math.cos(lat1) * math.cos(lat2)); 
    rad = 6371 # kilometers
    c = 2 * math.asin(math.sqrt(a)) 
    return rad * c # kilometers


def build_vehicle_distance_matrix_euclidean(positions):
    distance_matrix = np.zeros((len(positions), len(positions)), dtype=float)
    for i, row in enumerate(distance_matrix):
        for j, _ in enumerate(row):
            if i != j:
                first_drone = positions[find_name(i)]
                second_drone = positions[find_name(j)]
                distance_matrix[i, j] = math.sqrt((first_drone[0] - second_drone[0])**2 + 
                                                    (first_drone[1] - second_drone[1])**2 +
                                                    (first_drone[2] - second_drone[2])**2)
            else:
                distance_matrix[i, j] = 0
    return distance_matrix


def build_vehicle_distance_matrix_gps(drones: dict) -> np.array:
    distance_matrix = np.zeros((len(drones), len(drones)), dtype=float)
    for i, row in enumerate(distance_matrix):
        for j, column in enumerate(row):
            if i != j:
                first_drone = drones[find_name(i)].gps_pos_vec3
                second_drone = drones[find_name(j)].gps_pos_vec3
                distance_matrix[i, j] = round(haversine(
                    first_drone[0],
                    first_drone[1],
                    second_drone[0],
                    second_drone[1])*1000, 3)
            else:
                distance_matrix[i, j] = 0
    return distance_matrix