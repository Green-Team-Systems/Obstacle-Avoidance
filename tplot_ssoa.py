from threading import local
from turtle import color
from matplotlib import markers
import matplotlib.pyplot as plt
import numpy as np
import math as m
from utils.data_classes import PosVec3, MovementCommand, VelVec3
# vector, frame_pos, radius, ss_center

def pow(a):
    return a * a

def plot_new_point(point: PosVec3):
    ax.scatter(point.X, point.Y, point.Z, color = 'y', marker = '*', s=100)

def plot_sphere(sphere_pos: PosVec3, radius: float):
    u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    x = (radius) * np.cos(u)*np.sin(v) + sphere_pos.X
    y = (radius) * np.sin(u)*np.sin(v) + sphere_pos.Y
    z = (radius) * np.cos(v) + sphere_pos.Z
    ax.plot_wireframe(x,y,z)

def plot_line(pos_1: PosVec3, pos_2: PosVec3, mark):
   ax.plot([pos_1.X, pos_2.X], [pos_1.Y, pos_2.Y], [pos_1.Z, pos_2.Z], color = mark)

def plot_line_new(pos_1: PosVec3, pos_2: PosVec3, new_pos: PosVec3):
   ax.plot([new_pos.X, pos_2.X], [new_pos.Y, pos_2.Y], [new_pos.Z, pos_2.Z], color = 'g')
   ax.plot([pos_1.X, new_pos.X], [pos_1.Y, new_pos.Y], [pos_1.Z, new_pos.Z], color = 'g')


def collision_possability(wp_1: PosVec3, wp_2: PosVec3, obstacle_pos: PosVec3, radius_saftey: float):

    a = pow(wp_2.X  - wp_1.X) + pow(wp_2.Y  - wp_1.Y) + pow(wp_2.Z  - wp_1.Z)
    b = 2 * ((wp_2.X  - wp_1.X) * (wp_1.X  - obstacle_pos.X) + (wp_2.Y  - wp_1.Y) * (wp_1.Y  - obstacle_pos.Y) + (wp_2.Z  - wp_1.Z) * (wp_1.Z  - obstacle_pos.Z))
    c = pow(wp_1.X) + pow(wp_1.Y) + pow(wp_1.Z) + pow(obstacle_pos.X) + pow(obstacle_pos.Y) + pow(obstacle_pos.Z)
    c = c - 2 * (wp_1.X * obstacle_pos.X + wp_1.Y * obstacle_pos.Y + wp_1.Z * obstacle_pos.Z) - pow(radius_saftey)

    intersect = pow(b) - (4 * a * c)
    t1 = (-b - m.sqrt(m.fabs(intersect))) / (2.0 * a)
    t2 = (-b + m.sqrt(m.fabs(intersect))) / (2.0 * a)
    if intersect < 0 or t1 > 1 or t1 < 0 or t2 > 1 or t2 < 0:
        return False
    else:
        return True

def get_new_waypoint(wp1, wp2: PosVec3, obstacle_pos: PosVec3, radius_saftey: float):
    

    slope = (wp2.Y - wp1.Y) / (wp2.X - wp1.X)
    slope_inv = -1 / slope
    
    
    a = 1 + m.pow(slope_inv, 2)
    b = -2 * obstacle_pos.X + (2 * (slope_inv) * obstacle_pos.X)
    c = (m.pow(slope_inv, 2) * m.pow(obstacle_pos.X, 2)) + m.pow(obstacle_pos.X, 2) - m.pow(radius_saftey, 2)
    print(slope_inv)
    print("a: " + str(a))
    print("b: " + str(b))
    print("c: " + str(c))
    print(m.pow(b,2) - 4 * a * c)
    x1 = (-b + m.sqrt(m.pow(b, 2) - 4 * a * c)) / (2 * a)
    x2 = (-b - m.sqrt(m.pow(b, 2) - 4 * a * c)) / (2 * a)
    
    # y1 = m.sqrt(m.pow(radius_saftey, 2) - m.pow(x1 - obstacle_pos.X, 2)) + obstacle_pos.Y
    # y2 = m.sqrt(m.pow(radius_saftey, 2) - m.pow(x2 - obstacle_pos.X, 2)) + obstacle_pos.Y
    y1 = (slope_inv) * (x1 - obstacle_pos.X) + obstacle_pos.Y
    y2 = (slope_inv) * (x2 - obstacle_pos.X) + obstacle_pos.Y

    print("x1 y1", x1, y1)
    print("x2 y2", x2, y2)
    
    
    return [x1,y1, x2, y2]


    

wp_1 = PosVec3()
wp_2 = PosVec3()
new_wp = PosVec3()
obstacle_pos = PosVec3()
radius_saftey = 2
way_points = [[1, 1, 0], [10, 10,0], [20, 20, 1]]
obstacles = [[5, 5, 0], [15,15,15]]
ax = plt.axes(projection = '3d')
ax.set_xlim([0,20])
ax.set_ylim([0,20])
ax.set_zlim([0,20])
for x in range(1, len(way_points)):
    # set waypoint parameters
    wp_1.X = way_points[x - 1][0]
    wp_1.Y = way_points[x - 1][1]
    wp_1.Z = way_points[x - 1][2]
    wp_2.X = way_points[x][0]
    wp_2.Y = way_points[x][1]
    wp_2.Z = way_points[x][2]
    already_intersect = False
    for x in range(len(obstacles)):
        obstacle_pos.X = obstacles[x][0]
        obstacle_pos.Y = obstacles[x][1]
        obstacle_pos.Z = obstacles[x][2]
        # check for collision
        collision = collision_possability(wp_1, wp_2, obstacle_pos, radius_saftey)
        if collision == True:
            # calculate new waypoint
            [l_x1, l_y1, l_x2, l_y2] = get_new_waypoint(wp_1,wp_2, obstacle_pos, radius_saftey)
            # new_wp.X = point[0]
            # new_wp.Y = point[1]
            # new_wp.Z = point[2]
            plot_line(wp_1, wp_2, 'r')
            plot_sphere(obstacle_pos, radius_saftey)
            # plot_new_point(new_wp)
            # plot_line_new(wp_1, wp_2, new_wp)
            
            slope = (wp_2.Y - wp_1.Y) / (wp_2.X - wp_1.X)
            slope_inv = -1 / slope
            t = np.linspace(obstacle_pos.X - 5,obstacle_pos.X + 5,50)
            y = slope_inv * (t - obstacle_pos.X) + obstacle_pos.Y
            ax.plot(t, y, 'b', label = 'perpendicular line')
            ax.scatter(l_x1, l_y1, 0, color = 'y', marker = '*', s=100)
            ax.scatter(l_x2, l_y2, 0, color = 'g', marker = '*', s=100)
            
            already_intersect = True
        elif collision == False and already_intersect == True:
            plot_line(wp_1, wp_2, 'r')
        else:
            plot_line(wp_1, wp_2, 'g')
plt.show()