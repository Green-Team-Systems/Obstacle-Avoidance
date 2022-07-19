# -*- coding: utf-8 -*-
import numpy as np

def constrain(a, bottom, top):
    min = a if a < top else top
    max = min if min > bottom else bottom
    
    return max


def euler2RM(roll,pitch,yaw):
    R = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
    cr = np.cos(roll)
    sr = np.sin(roll)
    
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    
    cy = np.cos(yaw)
    sy = np.sin(yaw)
    
    R[0,0] = cp*cy
    R[1,0] = -cr*sy+sr*sp*cy
    R[2,0] = sr*sy+cr*sp*cy
    
    R[0,1] = cp*sy
    R[1,1] = cr*cy+sr*sp*sy
    R[2,1] = -sr*cy+cr*sp*sy
    
    R[0,2] = -sp
    R[1,2] = sr*cp
    R[2,2] = cr*cp
    
    return R.transpose()

def magnitude_sq(a, b, c):
    return (a*a+b*b+c*c)

def identity_matrix():
    R = np.array([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
    return R

def outer_product(a, b, c, d, e, f):
    R = np.array([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
    R[0, 0] = a * d
    R[1, 0] = a * e
    R[2, 0] = a * f

    R[0, 1] = b * d
    R[1, 1] = b * e
    R[2, 1] = b * f
    
    R[0, 2] = c * d
    R[1, 2] = c * e
    R[2, 2] = c * f

    return R


def skew_symetric(a, b, c):
    R = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
    R[0, 0] = 0.0
    R[1, 0] = -c
    R[2, 0] = b

    R[0, 1] = c
    R[1, 1] = 0.0
    R[2, 1] = -a
    
    R[0, 2] = -b
    R[1, 2] = a
    R[2, 2] = 0.0
    
    return R


def RotationMatrix_IwrtB(roll, pitch, yaw):
    Sk_q = skew_symetric(roll, pitch, yaw)
    final_result = identity_matrix() * (1 - magnitude_sq(roll, pitch, yaw)) + outer_product(roll, pitch, yaw, roll, pitch, yaw) * 2 \
        + Sk_q * 2 * 1

    return final_result
    