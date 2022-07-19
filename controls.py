import numpy as np
import math as m
from frame_utils import RotationMatrix_IwrtB, constrain, euler2RM

GRAVITY = 9.81
DRONE_MASS = 1
MOI = np.array([0.0066905, 0.00801, 0.0142525]) #Need to find a way to get these values or some how guess it!!!
MAX_THRUST = 4.179446268
MAX_TORQUE = 0.055562
PI = np.pi

class PIDController(object):
    
    def __init__(self):
        self.max_tilt = 1.0
        self.max_ascent_rate = 16
        self.max_descent_rate = 10
        self.Kp_yaw = 1.0
        self.Kp_r = 1.0
        self.Kp_roll = 0.2
        self.Kp_p = 1.0
        self.Kp_pitch = 0.2
        self.Kp_q = 1.0
        self.Kp_pos = 0.4
        self.Kp_vel = 1.0
        self.Kp_alt = 4.0
        self.Kp_hdot = 20.0
        self.Ki_alt = 0.1
        self.max_speed = 2.0
        self.integrator = 0
        self.last_error = 0
        
    def lateral_position_control(self, local_position_cmd: list, local_velocity_cmd: list, local_position: list, local_velocity: list, acceleration_ff = np.array([0.0, 0.0])):
        """Generate horizontal acceleration commands for the vehicle in the local frame """

        velocity_cmd = self.Kp_pos * (np.array(local_position_cmd) - np.array(local_position))
        
        velocity_norm = np.sqrt(velocity_cmd[0] * velocity_cmd[0] + \
                                velocity_cmd[1] * velocity_cmd[1])
        
        if velocity_norm > self.max_speed:
            velocity_cmd = velocity_cmd * self.max_speed / velocity_norm
        
        acceleration_cmd  = acceleration_ff + \
                            self.Kp_pos * (np.array(local_position_cmd) - np.array(local_position)) + \
                            self.Kp_vel * (np.array(local_velocity_cmd) - np.array(local_velocity))

        #print("Position: {}".format(np.array(local_position)))
        #print("Velocity Error: {}".format(velocity_cmd))

        return acceleration_cmd
    
    def altitude_control(self, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude, dt, acceleration_ff = 0.0):
        """Generate vertical acceleration (thrust) command """

        error = altitude_cmd - altitude
        print("Error Cmd: {}".format(error))
        p_term = self.Kp_alt * (error)
        print("vertical_velocity_cmd Cmd: {}".format(vertical_velocity_cmd))
        velocity_error = p_term - self.last_error
        self.integrator += error * dt
        d_term = self.Kp_hdot * velocity_error
        i_term = self.Ki_alt * self.integrator

        rotation_matrix = RotationMatrix_IwrtB(attitude[0], attitude[1], attitude[2])
        b_z = rotation_matrix[2,2]

        comb = p_term + d_term + i_term + acceleration_ff
        print("B_Z Cmd: {}".format(b_z))
        accel_cmd = - ( comb - GRAVITY) / b_z
        print("Accel Cmd: {}".format(accel_cmd))
        self.last_error = p_term
        thrust = DRONE_MASS * constrain(accel_cmd, self.max_descent_rate, self.max_ascent_rate)
        print("Thrust: {}".format(thrust))   #### Big notes we calculate in Newtons but the drone flies in thrust pounds need to do conversions to make sure the stuff works properly

        return thrust
    
    def roll_pitch_controller(self, acceleration_cmd, attitude, thrust_cmd):
        """ Generate the rollrate and pitchrate commands in the body frame """
        R = euler2RM(attitude[0], attitude[1], attitude[2])
        c_d = thrust_cmd / DRONE_MASS

        if thrust_cmd > 0.0: 
            target_R13 = -np.clip(acceleration_cmd[0].item()/c_d, -self.max_tilt, self.max_tilt) #-min(max(acceleration_cmd[0].item()/c_d, -self.max_tilt), self.max_tilt)
            target_R23 = -np.clip(acceleration_cmd[1].item()/c_d, -self.max_tilt, self.max_tilt) #-min(max(acceleration_cmd[1].item()/c_d, -self.max_tilt), self.max_tilt)
            
            p_cmd = (1/R[2, 2]) * \
                    (-R[1, 0] * self.Kp_roll * (R[0, 2]-target_R13) + \
                     R[0, 0] * self.Kp_pitch * (R[1, 2]-target_R23))
            q_cmd = (1/R[2, 2]) * \
                    (-R[1, 1] * self.Kp_roll * (R[0, 2]-target_R13) + \
                     R[0, 1] * self.Kp_pitch * (R[1, 2]-target_R23))
        else:
            print(f"negative thrust command {thrust_cmd}")
            p_cmd = 0.0
            q_cmd = 0.0
            thrust_cmd = 0.0
        return np.array([p_cmd, q_cmd])

    def body_rate_control(self, body_rate_cmd, body_rate):
        """ Generate the roll, pitch, yaw moment commands in the body frame """
        Kp_rate = np.array([self.Kp_p, self.Kp_q, self.Kp_r])
        rate_error = body_rate_cmd - body_rate
        
        moment_cmd = MOI * np.multiply(Kp_rate, rate_error)
        if np.linalg.norm(moment_cmd) > MAX_TORQUE:
            moment_cmd = moment_cmd*MAX_TORQUE/np.linalg.norm(moment_cmd)
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
        return yawrate_cmd