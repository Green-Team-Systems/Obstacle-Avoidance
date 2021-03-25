# ===============================================================
# Created By: Tyler Fedrizzi
# Authors: Tyler Fedrizzi
#          Rohan Potdar
# Created On: September 6th, 2020
# Last Modified: March 21st, 2021
# 
# Description: Drone class to hold all details of the individual drone.
# ===============================================================
from comms import Comms
from multiprocessing import Process
from queue import Queue
import psutil
import os
from utils.killer_utils import GracefulKiller
# caller is responsible for creating PORTS (see below), terminating drones, calling KillerClient

class Drone(Process):
    _timeout = 0.01

    def __init__(self, options, PORTS, ID=0, speed=5, position=[0,0,0]):
        # Note: PORTS = mp.Array(ctypes.c_wchar_p, num_drones) should be initialized by main and passed to each Drone
        Process.__init__(self, daemon=True)
        # Decrease priority of Drone Process; otherwise main doesn't get enough run time
        if os.name == 'nt':
            p = psutil.Process(pid=self.pid)
            p.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
        if os.name == 'posix':
            os.nice(5)
        self.model = options['drone_model']
        # In the future, this option will be passed to a method to select our controller code.
        self.controller = options['controller']        # SimpleFlight is the default for AirSim
        # Default will be carrot-stick (standard in AirSim). Upgrade will be PX4
        self.autopilot = options['autopilot']          # Carrot-Stick method for AirSim
        # This will load the collision avoidance module. In the future, this will be a class that
        # can be defined by the user.
        self.collision_avoidance = options['collision_avoidance']
        # User input for communications is passed directly to the communication system
        self.comms = Comms(ID=ID, PORTS=PORTS, transmit_distance=options['communication_distance'],
                                 transmit_modulation=options['communication_modulation'], num_drones=options['num_drones'])
        self.weight = 2                     # kilograms
        self.pos_vec3 = [0,0,0]             # X, Y, Z where Z is down / This is the commanded position that can be updated by the user
        self.real_pos_vec3 = [0,0,0]        # Actual position of the drone that the user doesn't have access to
        self.gps_pos_vec3 = [0,0,0]         # Latitude, Longitude, Altitude
        self.init_pos_vec3 = position       # X, Y, Z where Z is down
        self.current_velocity = [0,0,0]     # Meters / second
        self.orientation = [0,0,0,0]        # Quaternion Orientation
        self.ID = ID                        # Integer
        self.speed = speed
        self.name = "Drone" + str(self.ID)  # Needed for AirSim
        self.forward_speed = speed          # The default forward speed relative to the vehicle reference frame (0 degree)
        self.map = None                     # Map of the environment you're operating
    
    # Following the Python PEP8 style guide, we use snake case for method names.
    def get_ned_position(self):
        """
            The the North, East, Down position of the drone.
        """
        return self.pos_vec3

    def set_position(self, vec3, mode = ""):
        # TODO: Implement conversions between NED and GPS
        if not (mode == "" or mode == "gps"):
            print("Mode = \"\",\"gps\" only, Position unchanged")
            return self.get_ned_position()
        if mode == "":
            setattr(self, "pos_vec3", vec3)
            # setattr(self, "gps_pos_vec3", ned2gps(vec3))
        else:
            setattr(self, "gps_pos_vec3", vec3)
            # setattr(self, "gps_pos_vec3", gps2ned(vec3))
        return self.get_ned_position()
    
    def update_ned_position(self, vec3: dict) -> None:
        """
            Update the NED coordinate vector with positions. This is required
            both for setting the initial position vector and for updating the NED
            coordinate vector.
        """
        self.pos_vec3 = [vec3['X'], vec3['Y'], vec3['Z']]
    
    def start(self):
        # Initialization; wait for comms; etc.
        pass

    def run(self):
        killer = GracefulKiller
        try:
            while not killer.kill_now:
                pass            # Main execution loop; CV, Lidar, everything
        except Exception as error:
            print(error)
        finally:
            # Cleanup
            self.comms.terminate()

class Controller(Drone):

    def __init__(self):
        self.obstacle_queue = Queue()
        self.action_queue = Queue()
    
    def check_queue(self):
        try:
            message = self.obstacle_queue.get(timeout=Drone._timeout)
            if message:
                # Do something
                pass
            action = self.action_queue.get(timeout=Drone._timeout)
            if action:
                # Take that action
                self.route_plan([0,0,0], [1,2,3])

        except Exception as error:
            print(error)

    def route_plan(self, target_location, current_location):
        self.map
        # make a plan
        # Return my plan
        return True
    
