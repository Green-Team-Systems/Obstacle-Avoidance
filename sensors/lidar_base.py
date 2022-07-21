# TODO Add a SensorBase class to handle health checks for sensors
import pickle

from numpy import float16, ndarray, identity, save, zeros


class LiDARBase():
    """
    Base LiDAR class containing the initialization methods and required
    processing functionality for follow-on LiDAR systems.

    ## Inputs:
    - sensor_name [str] unique name of the lidar sensor
    """
    def __init__(self,
                 sensor_name: str,
                 sensor_type: str = "Velodyne Puck",
                 file_type: str = "numpy",
                 sensor_position: list = [0.0, 0.0, 0.0]) -> None:
        self.sensor_name = sensor_name
        # Initial coordinate frame is identity 4x4 matrix to represent
        # a complete alignment of data with body frame
        self.coordinate_frame = identity(4)
        self.coordinate_frame[0][-1] = sensor_position[0]
        self.coordinate_frame[1][-1] = sensor_position[1]
        self.coordinate_frame[2][-1] = sensor_position[2]
        self.file_type = file_type
        self.sensor_data = zeros((256, 256), dtype=float16)
        self.lidar_info = LidarInfo(name=sensor_type)
        self.cloud = None

    def process_data(self) -> ndarray:
        raise NotImplementedError("Please implement this method!")

    def get_lidar_data(self) -> list:
        raise NotImplementedError("Please implement this method")

    def save_data(self, filename) -> None:
        if self.file_type == "numpy":
            with open("{}.npy".format(filename), "w") as file:
                save(file, self.sensor_data)
        elif self.file_type == "pickle":
            with open("{}.pickle".format(filename), "w") as file:
                pickle.dump(self.sensor_data, file)


class LidarInfo():
    """
    Class holding the information about each individual Lidar that is
    replicated by the SWARM system in AirSim.

    ## Inputs:
    - name [str] The name and type of lidar that is used
    """
    def __init__(self, name: str) -> None:
        self.fill_in_info(name)

    def fill_in_info(self, name: str) -> None:
        """
        Add the required data members for the specific Lidar found.

        Main parameters set:
        - Range [Meters]
        - Number of Channels [Int]
        - Points per Second [Int]
        - Horizontal FOV [Degrees]
        - Vertical FOV [Degrees]
        - Update Rate [hertz]
        - Weight [grams]

        ## Inputs:
        - name [str] The name of the Lidar
        """
        if name == "Velodyne Puck":
            self.range = 100.0
            self.number_of_channels = 16
            self.points_per_second = 300000
            self.horizontal_fov = 360.0  # Degrees
            self.vertical_fov = 45.0  # Degrees
            self.power_consumption = 8.0  # Watts
            self.update_rate = [(5.0 + i) for i in range(0, 16)]
            self.weight = 590.0  # grams