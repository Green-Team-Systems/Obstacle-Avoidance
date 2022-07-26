from threading import local
from turtle import color
from matplotlib import markers
import matplotlib.pyplot as plt
import numpy as np
from utils.data_classes import PosVec3, MovementCommand, VelVec3
# vector, frame_pos, radius, ss_center

def pow(a):
    return a * a


def plot_intersect(pos_1: PosVec3, pos_2: PosVec3, sphere_pos: PosVec3, radius: float, point: list):
    fig = plt.figure()
    ax = plt.axes(projection = '3d')
    ax.set_xlim([0,10])
    ax.set_ylim([0,10])
    ax.set_zlim([0,10])
    ax.scatter(point[0], point[1], point[2],color = 'y', marker = '*', s=100)
    ax.plot([pos_1.X, pos_2.X], [pos_1.Y, pos_2.Y], [pos_1.Z, pos_2.Z], color = 'r')
    ax.plot([point[0], pos_2.X], [point[1], pos_2.Y], [point[2], pos_2.Z], color = 'g')
    ax.plot([pos_1.X, point[0]], [pos_1.Y, point[1]], [pos_1.Z, point[2]], color = 'g')

    u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    x = radius * np.cos(u)*np.sin(v) + sphere_pos.X
    y = radius * np.sin(u)*np.sin(v) + sphere_pos.Y
    z = radius * np.cos(v) + sphere_pos.Z
    ax.plot_wireframe(x,y,z)

    
    
    plt.show()

def collision_possability(wp_1: PosVec3, wp_2: PosVec3, obstacle_pos: PosVec3, radius_saftey: float):

    a = pow(wp_2.X  - wp_1.X) + pow(wp_2.Y  - wp_1.Y) + pow(wp_2.Z  - wp_1.Z)
    b = 2 * ((wp_2.X  - wp_1.X) * (wp_1.X  - obstacle_pos.X) + (wp_2.Y  - wp_1.Y) * (wp_1.Y  - obstacle_pos.Y) + (wp_2.Z  - wp_1.Z) * (wp_1.Z  - obstacle_pos.Z))
    c = pow(wp_1.X) + pow(wp_1.Y) + pow(wp_1.Z) + pow(obstacle_pos.X) + pow(obstacle_pos.Y) + pow(obstacle_pos.Z)
    c = c - 2 * (wp_1.X * obstacle_pos.X + wp_1.Y * obstacle_pos.Y + wp_1.Z * obstacle_pos.Z) - pow(radius_saftey)

    intersect = pow(b) - (4 * a * c)
    if intersect >= 0:
        return True
    else:
        return False

def get_new_waypoint(wp: PosVec3, obstacle_pos: PosVec3, radius_saftey: float):
    radius_saftey = radius_saftey + 1.5

    u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    x = radius_saftey * np.cos(u)*np.sin(v) + obstacle_pos.X
    y = radius_saftey * np.sin(u)*np.sin(v) + obstacle_pos.Y
    z = radius_saftey * np.cos(v) + obstacle_pos.Z
    # x = np.array((1, 2, 3))
    # y = np.array((1, 2, 3))
    # z = np.array((1, 2, 3))
    a = np.dstack((x, y, z))
    b = np.array((wp.X, wp.Y, wp.Z))
    c = a - b
    dist = np.linalg.norm(c, axis=-1)
    min = np.nanargmin(dist, axis = 1)
    location = a[0,min]
    
    return location[0]


wp_1 = PosVec3()
wp_1.X = 1
wp_1.Y = 1
wp_1.Z = 1
wp_2 = PosVec3()
wp_2.X = 10
wp_2.Y = 10
wp_2.Z = 1
obstacle_pos = PosVec3()
obstacle_pos.X = 5
obstacle_pos.Y = 5
obstacle_pos.Z = 3

radius_saftey = 1

collision = collision_possability(wp_1 , wp_2, obstacle_pos, radius_saftey)
if collision == True:
    point = get_new_waypoint(wp_2, obstacle_pos, radius_saftey)
else:
    point = [wp_2.X, wp_2.Y, wp_2.Z ]
print(f'Drone Collision: {collision}')
plot_intersect(wp_1 , wp_2, obstacle_pos, radius_saftey, point)
