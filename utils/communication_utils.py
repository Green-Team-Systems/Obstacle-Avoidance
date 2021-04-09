from __future__ import print_function
import numpy as np
from utils.name_utils import find_name
from sys import getsizeof, stderr
from itertools import chain
from collections import deque
try:
    from reprlib import repr
except ImportError:
    pass
import config

def find_drone(target):
    """ Returns drone with self.ID = target """
    for drone in config.drone_list:
        if drone.ID == target:
            return drone
    return None

def total_size(o, handlers={}, verbose=False):  # Copied this from SO
    """ Returns the approximate memory footprint an object and all of its contents.

    Automatically finds the contents of the following builtin containers and
    their subclasses:  tuple, list, deque, dict, set and frozenset.
    To search other containers, add handlers to iterate over their contents:

        handlers = {SomeContainerClass: iter,
                    OtherContainerClass: OtherContainerClass.get_elements}

    """
    dict_handler = lambda d: chain.from_iterable(d.items())
    all_handlers = {tuple: iter,
                    list: iter,
                    deque: iter,
                    dict: dict_handler,
                    set: iter,
                    frozenset: iter,
                   }
    all_handlers.update(handlers)     # user handlers take precedence
    seen = set()                      # track which object id's have already been seen
    default_size = getsizeof(0)       # estimate sizeof object without __sizeof__

    def sizeof(o):
        if id(o) in seen:       # do not double count the same object
            return 0
        seen.add(id(o))
        s = getsizeof(o, default_size)

        if verbose:
            print(s, type(o), repr(o), file=stderr)

        for typ, handler in all_handlers.items():
            if isinstance(o, typ):
                s += sum(map(sizeof, handler(o)))
                break
        return s

    return sizeof(o)

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