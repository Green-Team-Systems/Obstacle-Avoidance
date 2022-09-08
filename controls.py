import numpy as np
import math as m
from utils.frame_utils import RotationMatrix_IwrtB, constrain, magnitude

GRAVITY = 9.81
DRONE_MASS = 1
MOI = np.array([0.0066905, 0.00801, 0.0142525]) #Need to find a way to get these values or some how guess it!!!
MAX_THRUST = 4.179446268
MAX_TORQUE = 0.055562
PI = np.pi

class PIDController(object):
    
    def __init__(self):
        self.Kp_pos_x = 0.08
        self.Kp_pos_y = 0.08
        self.Kp_pos = np.array([self.Kp_pos_x, self.Kp_pos_y])
        self.Kp_vel = 3.0
        self.Kd_vel = 1.0
        self.Kp_bank = 0.1
        self.Kp_yaw = 0.1
        self.Kp_p = 0 # controls left and right
        self.Kp_q = 1 # controls forward and backwards
        self.Kp_r = 0 # yaw
        self.max_tilt = 3
        self.max_ascent_rate = 16
        self.max_descent_rate = 10
        self.Kp_alt = 0.2
        self.Kd_alt = 0.8
        self.Ki_alt = 0.00
        self.max_speed = 2
        self.max_accel = 0.5
        self.integrator = 0
        self.last_error_pos = 0
        self.last_error_alt = 0
        
    def lateral_position_control(self, local_position_cmd: list, local_velocity_cmd: list, local_position: list, local_velocity: list, dt, acceleration_ff = np.array([0.0, 0.0])):
        """Generate horizontal acceleration commands for the vehicle in the local frame """

        velocity_cmd = np.multiply(self.Kp_pos, (np.array(local_position_cmd) - np.array(local_position)))
        velocity_norm = np.sqrt(velocity_cmd[0] * velocity_cmd[0] + \
                                velocity_cmd[1] * velocity_cmd[1])
        if magnitude(velocity_cmd[0], velocity_cmd[1], 1) > self.max_speed:
            velocity_cmd = (velocity_cmd / velocity_norm) * self.max_speed

        vel_err = (np.array(velocity_cmd) - np.array(local_velocity))
        der = (vel_err - self.last_error_pos) / dt
        acceleration_cmd  = acceleration_ff + self.Kp_vel * vel_err + self.Kd_vel  * der
        print(f'vel err: {self.Kp_vel * (np.array(velocity_cmd) - np.array(local_velocity))}\n')
        if magnitude(acceleration_cmd[0], acceleration_cmd[1], 1) > self.max_accel:
            acceleration_cmd = (acceleration_cmd / magnitude(acceleration_cmd[0], acceleration_cmd[1], 1)) * self.max_accel
        acceleration_cmd[0] = -acceleration_cmd[0]
        self.last_error_pos = vel_err        
        return acceleration_cmd
    
    def altitude_control(self, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude, dt, acceleration_ff = 0.0):
        """Generate vertical acceleration (thrust) command """
        error = -1 * (altitude_cmd - altitude)
        
        #print("Altitude Error Cmd: {}".format(error))
        p_term = self.Kp_alt * (error)
        #print("vertical_velocity_cmd Cmd: {}".format(vertical_velocity_cmd))
        velocity_error = (error - self.last_error_alt) / dt  # Create a derivative term
        self.integrator += error * dt
        d_term = self.Kd_alt * velocity_error
        i_term = self.Ki_alt * self.integrator

        rotation_matrix = RotationMatrix_IwrtB(attitude[0], attitude[1], attitude[2])
        b_z = rotation_matrix[2,2]
        #print("P: {} I: {} D: {}".format(p_term, i_term, d_term))
        comb = -1 * ((p_term + d_term + i_term * dt * dt) + 0.0)
        # print("B_Z Cmd: {}".format(b_z))
        #print("Combined Thrust: {}".format(comb))
        hover_thrust = GRAVITY + 1.22102
        accel_cmd = -(comb - hover_thrust) / b_z
        # accel_cmd = GRAVITY + 1.22102
        
        #print("Accel Cmd: {}".format(accel_cmd))
        self.last_error_alt = error
        thrust = DRONE_MASS * constrain(accel_cmd, self.max_descent_rate, self.max_ascent_rate)
        # print("Thrust: {}".format(thrust))   #### Big notes we calculate in Newtons but the drone flies in thrust pounds need to do conversions to make sure the stuff works properly

        return thrust
    
    def roll_pitch_controller(self, acceleration_cmd, attitude, thrust_cmd):
        """ Generate the rollrate and pitchrate commands in the body frame """
        rotation_matrix = RotationMatrix_IwrtB(attitude[0], attitude[1], attitude[2])
        if(thrust_cmd > 0):
            thrust = - thrust_cmd / DRONE_MASS
            min = -np.sin(self.max_tilt) 
            max = np.sin(self.max_tilt) 
            b_x = constrain((acceleration_cmd[0] / thrust), min, max)
            b_y = constrain((acceleration_cmd[1] / thrust), min, max)
            b_x_d = self.Kp_bank * (b_x - rotation_matrix[0,2])
            b_y_d = self.Kp_bank * (b_y - rotation_matrix[1,2])

            p_cmd = (rotation_matrix[1,0] * b_x_d - rotation_matrix[0,0] * b_y_d) / rotation_matrix[2,2]
            q_cmd = (rotation_matrix[1,1] * b_x_d - rotation_matrix[0,1] * b_y_d) / rotation_matrix[2,2]
        else:
            p_cmd = 0.0
            q_cmd = 0.0

        return np.array([p_cmd, q_cmd])

    def body_rate_control(self, body_rate_cmd, body_rate):
        """ Generate the roll, pitch, yaw moment commands in the body frame """
        Kp_rate = np.array([self.Kp_p, self.Kp_q, self.Kp_r])
        rate_error = body_rate_cmd - body_rate

        print("Rate Error: {}".format(rate_error))
        k_p = np.multiply(rate_error, Kp_rate)
        print("K_p: {}".format(k_p))
        moment_cmd = np.multiply(k_p, MOI)


        return moment_cmd

    def yaw_control(self, yaw_cmd, yaw):
        """ Generate the target yawrate """
        yaw_cmd = np.mod(yaw_cmd, 2.0*PI)
        
        yaw_error = yaw_cmd - yaw
        if yaw_error > PI:
            yaw_error = yaw_error - 2.0*PI
        elif yaw_error < -PI:
            yaw_error = yaw_error + 2.0*PI
        
        yawrate_cmd = self.Kp_yaw * yaw_error
        return -yawrate_cmd