import airsim
import sys
import math
import time
import argparse
import pprint


class Drone_Obstacle_Avoidance:
    def __init__(self):
        # Connect to the AirSim simulator
        self.client = airsim.MultirotorClient()
        self.starting_pos = (
            self.client.getMultirotorState().kinematics_estimated.position
        )
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.timer = 0

    def parse_liadar_date(self, lidarData):
        if len(lidarData) < 3:
            print("\tNo points received from Lidar data")

        else:
            # Divides the lidarData into 3 seperate variables for x, y and z
            y_points_last = 0
            length = len(lidarData)
            overall_point_list = list()
            next_row = list()
            for i in range(0, length, 3):
                xyz = lidarData[i : i + 3]
                if xyz[1] != math.fabs(xyz[1]) and y_points_last == math.fabs(
                    y_points_last
                ):
                    overall_point_list.append(next_row)
                    next_row = list()
                next_row.append(xyz)
                y_points_last = xyz[1]
        return overall_point_list

    def move_forward(self):
        self.client.moveByVelocityAsync(2, 0, 0, math.inf)

    def move_right(self):
        self.client.moveByVelocityAsync(0, 2, 0, math.inf)

    def move_up(self):
        self.client.moveByVelocityAsync(0, 0, -5, math.inf)

    def file_debug(self, point_cloud, new_point_cloud, reduced_point_cloud):
        file = open("lidardatavehicle.txt", "a")

        file.write("\nLidar Data:\n")
        for x in point_cloud:
            for i in x:
                file.write(str(i))
            file.write("\n")

        height = self.client.getDistanceSensorData()
        drone_height = height.distance
        file.write("\nDrone Height: \n")
        file.write(str(drone_height))
        file.write("\n")

        file.write("\nUpdated Lidar Data:\n")
        for x in new_point_cloud:
            for i in x:
                file.write(str(i))
            file.write("\n")

        file.write("\nReduced Lidar Data:\n")
        for x in reduced_point_cloud:
            for i in x:
                file.write(str(i))
            file.write("\n")

    def slope_calculation(self, point_cloud, drone_velocity):
        if len(point_cloud) < 2:
            return 0, 0, 0

        try:
            midpoint_top_level = int(len(point_cloud[1]) / 2)
            x2_distance = point_cloud[1][midpoint_top_level][0]
            z2_distance = -point_cloud[1][midpoint_top_level][2]
            bottom_level_point = len(point_cloud) - 1
            midpoint_bottom_level = int(len(point_cloud[bottom_level_point]) / 2)
            x1_distance = point_cloud[bottom_level_point][midpoint_bottom_level][0]
            z1_distance = -point_cloud[bottom_level_point][midpoint_bottom_level][2]

            x_distance = math.fabs(x2_distance - x1_distance)
            z_distance = math.fabs(z2_distance - z1_distance)

            # Given the distances, do rise over run
            slope = math.atan2(z_distance, x_distance)  # Radians
            slope = math.degrees(slope)
            self.previous_slope = slope

            hypo = math.sqrt(math.pow(x_distance, 2) + math.pow(z_distance, 2))

            z_velocity = z_distance / hypo
            x_velocity = x_distance / hypo
            z_velocity = z_velocity * drone_velocity
            x_velocity = x_velocity * drone_velocity

            return slope, x_velocity, z_velocity

        except Exception:
            return 0, 0, 0

    def remove_ground_points(self, point_cloud, drone_height):
        bottom_level = len(point_cloud) - 1
        drone_tolernace_high = drone_height + 0.4
        drone_tolernace_low = drone_height - 2
        for x in range(bottom_level, 0, -1):
            z_point = point_cloud[x][0][2]
            if z_point >= drone_tolernace_low and z_point <= drone_tolernace_high:
                del point_cloud[x]
        return point_cloud

    def reduce_window_size(self, point_cloud, min_window, max_window):
        row = []
        point_cloud_reduced = []
        for _ in point_cloud:
            for x in _:
                y_point = x[1]
                if y_point > min_window and y_point < max_window:
                    row.append(x)
            point_cloud_reduced.append(row)

        return point_cloud_reduced

    def object_detection(self, point_cloud, threshold_distance):
        for _ in point_cloud:
            for i in _:
                x_point = i[0]
                if x_point <= threshold_distance:
                    return True

        return False

    def obstacle_avoidance(
        self,
        point_cloud,
        drone_height,
        min_incline,
        max_incline,
        drone_velocity,
        min_window,
        max_window,
        threshold_distance,
        threshold_timer,
    ):
        # Calculate the slope and potential new x and z velocity
        slope, x_velocity, z_velocity = self.slope_calculation(
            point_cloud, drone_velocity
        )

        # If slope is in between the range move by the new velocity
        if slope > min_incline and slope <= max_incline:
            self.client.moveByVelocityAsync(x_velocity, 0, -z_velocity, math.inf)

        else:
            # Remove Ground Points from point cloud
            new_point_cloud = self.remove_ground_points(point_cloud, drone_height)

            # Reduce the new point cloud to a certain window size
            reduced_point_cloud = self.reduce_window_size(
                new_point_cloud, min_window, max_window
            )

            # Use the new Point Cloud to see if an object is dected
            object_detected = self.object_detection(
                reduced_point_cloud, threshold_distance
            )

            # If detected then run Right Hand Rule
            if object_detected == True:
                self.move_right()

                # Start a timer or a distance counter have not determined which one is better over the other
                self.timer = self.timer + 0.01

                # If that timer / counter is exceded then send the drone straight up till object is not detected
                if self.timer > threshold_timer:
                    self.move_up()

            # Else continue on
            else:
                self.timer = 0
                # Move forward command for now during integrationg this might be removed
                self.move_forward()

    def execute(self):
        """
        Executes File
        """

        min_incline = 10
        max_incline = 25
        min_window = -1.0
        max_window = 1.0
        threshold_distance = 7
        threshold_time = 1
        drone_velocity = 3
        print("arming the drone...")
        self.client.armDisarm(True)

        # Takeoff of Drone
        state = self.client.getMultirotorState()
        s = pprint.pformat(state)
        print(s)
        airsim.wait_key("Press any key to takeoff")
        self.client.takeoffAsync().join()
        state = self.client.getMultirotorState()

        # Waitkey to begin Lidar drone testing.
        airsim.wait_key("Press any key to get Lidar readings")
        self.client.hoverAsync().join()

        while 1:
            # Lidar_data and data both need to be placed in a while loop along with function defination to make sure it works :)
            height = self.client.getDistanceSensorData()
            drone_height = height.distance
            lidar_data = self.client.getLidarData()
            data = lidar_data.point_cloud
            point_cloud = self.parse_liadar_date(data)

            """ The below lines should be uncommented only if file debug is needed """
            # new_point_cloud = self.remove_ground_points(point_cloud, drone_height)
            # reduced_point_cloud = self.reduce_window_size(new_point_cloud, -1.0, 1.0)
            # self.file_debug(point_cloud, new_point_cloud, reduced_point_cloud)

            """ 
            Apperantly there is an issue with linear velocity call in Airsim https://github.com/microsoft/AirSim/issues/2914
            discuss potential fixes with Tyler. For now keep it constant
            """
            # drone_kinematics = self.client.getMultirotorState().kinematics_estimated
            # drone_linear_velocity = drone_kinematics.linear_velocity

            self.obstacle_avoidance(
                point_cloud,
                drone_height,
                min_incline,
                max_incline,
                drone_velocity,
                min_window,
                max_window,
                threshold_distance,
                threshold_time,
            )

            time.sleep(0.01)

    def stop(self):
        airsim.wait_key("Press any key to reset to original state")

        self.client.armDisarm(False)
        self.client.reset()

        self.client.enableApiControl(False)
        print("Done!\n")


# main
if __name__ == "__main__":
    args = sys.argv
    args.pop(0)
    arg_parser = argparse.ArgumentParser("Lidar.py makes drone fly and gets Lidar data")

    arg_parser.add_argument(
        "-save-to-disk", type=bool, help="save Lidar data to disk", default=False
    )

    args = arg_parser.parse_args(args)
    drone_oa = Drone_Obstacle_Avoidance()
    try:
        drone_oa.execute()
    except Exception as error:
        print(error)
    finally:
        drone_oa.stop()
