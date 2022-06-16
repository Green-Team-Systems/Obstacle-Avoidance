import numpy as np
import math as m
from frame_utils import euler2RM

GRAVITY = -9.81
DRONE_MASS = 1.0
MOI = np.array([0.0, 0.0, 0.0]) #Need to find a way to get these values or some how guess it!!!
MAX_THRUST = 4.179446268
MAX_TORQUE = 0.055562
PI = np.pi

class PIDController(object):
    
    def pid_config(self, t_rise, delta):
        w = 1 / (1.57 * t_rise)
        return w * w, 2  * delta * w
    
    def __init__(self):
        
        self.body_rate_kP = np.array([0.0, 0.0, 0.0])
        self.altitude_kP, self.altitude_kD = self.pid_config(0.0, 0.0)
        self.yaw_kP = 0.0

        self.rpy_kP_Roll = 0.0
        self.rpy_kD_Pitch = 0.0

        self.lateral_kP, self.lateral_kD = self.pid_config(0.0, 0.0)
        return
    
    def trajectory_follower(self, position_trajectory, yaw_trajectory, time_trajectory, current_time):
        
        ind_min = np.argmin(np.abs(np.array(time_trajectory) - current_time))
        time_ref = time_trajectory[ind_min]


        if current_time < time_ref:
            position0 = position_trajectory[ind_min - 1]
            position1 = position_trajectory[ind_min]

            time0 = time_trajectory[ind_min - 1]
            time1 = time_trajectory[ind_min]
            yaw_cmd = yaw_trajectory[ind_min - 1]

        else:
            yaw_cmd = yaw_trajectory[ind_min]
            if ind_min >= len(position_trajectory) - 1:
                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min]

                time0 = 0.0
                time1 = 1.0
            else:

                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min + 1]
                time0 = time_trajectory[ind_min]
                time1 = time_trajectory[ind_min + 1]

        position_cmd = (position1 - position0) * \
                        (current_time - time0) / (time1 - time0) + position0
        velocity_cmd = (position1 - position0) / (time1 - time0)

        return (position_cmd, velocity_cmd, yaw_cmd)
        
    def lateral_position_control(self, local_position_cmd, local_velocity_cmd, local_position, local_velocity, acceleration_ff = np.array([0.0, 0.0])):
        """Generate horizontal acceleration commands for the vehicle in the local frame """

        err_p = local_position_cmd - local_position
        err_dot = local_velocity_cmd - local_velocity

        return self.lateral_kP * err_p + self.lateral_kD * err_dot + acceleration_ff
    
    def altitude_control(self, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude, acceleration_ff = 0.0):
        """Generate vertical acceleration (thrust) command """

        z_err = altitude_cmd - altitude
        z_err_dot = vertical_velocity_cmd - vertical_velocity
        b_z = euler2RM(*attitude)[2,2]

        u_1 = self.altitude_kP * z_err + self.altitude_kD * z_err_dot + acceleration_ff
        acc = (u_1 - GRAVITY)/b_z

        thrust = DRONE_MASS * acc
        if thrust > MAX_THRUST:
            thrust = MAX_THRUST
        else:
            if thrust < 0.0:
                thrust = 0.0

        return thrust
    
    def roll_pitch_controller(self, acceleration_cmd, attitude, thrust_cmd):
        """ Generate the rollrate and pitchrate commands in the body frame """

        if thrust_cmd > 0.:
            c = - thrust_cmd / DRONE_MASS;
            b_x_c, b_y_c = np.clip(acceleration_cmd / c, -1., 1)

            rot_mat = euler2RM(*attitude)

            b_x = rot_mat[0, 2]
            b_x_err = b_x_c - b_x
            b_x_p_term = self.rpy_kP_Roll * b_x_err

            b_y = rot_mat[1,2]
            b_y_err = b_y_c - b_y
            b_y_p_term = self.rpy_kD_Pitch * b_y_err

            b_x_commanded_dot = b_x_p_term
            b_y_commanded_dot = b_y_p_term

            rot_mat1=np.array([[rot_mat[1,0],-rot_mat[0,0]],[rot_mat[1,1],-rot_mat[0,1]]])/rot_mat[2,2]

            rot_rate = np.matmul(rot_mat1, np.array([b_x_commanded_dot,b_y_commanded_dot]).T)
            p_c = rot_rate[0]
            q_c = rot_rate[1]
            # print(f'{b_x_err} {b_y_err}')
            return np.array([p_c, q_c])
        else:
            return np.array([0., 0.])

    def body_rate_control(self, body_rate_cmd, body_rate):
        """ Generate the roll, pitch, yaw moment commands in the body frame """

        taus = MOI * np.multiply(self.body_rate_kP, ( body_rate_cmd - body_rate ))

        taus_mod = np.linalg.norm(taus)

        if taus_mod > MAX_TORQUE:
            taus = taus * MAX_TORQUE / taus_mod

        return taus

    def yaw_control(self, yaw_cmd, yaw):
        """ Generate the target yawrate """

        yaw_error = yaw_cmd - yaw
        if yaw_error > PI:
            yaw_error = yaw_error - 2.0 * PI
        elif yaw_error < -PI:    
            yaw_error = yaw_error + 2.0 * PI

        yawrate_cmd = self.yaw_kP * yaw_error
        return yawrate_cmd
