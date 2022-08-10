from dis import dis
from re import X
from tkinter import Y
from turtle import pos
from matplotlib import projections
import matplotlib.pyplot as plt
import numpy as np
import math as m
from numpy import array, sin, cos
from itertools import product, combinations

import numpy
from utils.data_classes import PosVec3, MovementCommand, VelVec3

def pow(a):
    return a * a

def magnitude(a):
    return np.sqrt(np.dot(a, a))

def closest(wp, point: PosVec3, radius):
    wp = np.array(wp)
    x = [point.X - radius, point.X + radius]
    y = [point.Y - radius, point.Y + radius]
    z = [point.Z - radius, point.Z + radius]
    min = m.inf
    pts = np.stack((x, y, z), 0)
    vertices = (np.array(np.meshgrid(*pts)).T).reshape(2**3,3)
    for x in vertices:
        dist = np.linalg.norm(x - wp)
        if dist < min: 
            min = dist
            i = x  
    return i

def rotation_angle(cube_point, wp_1, wp_2):
    wp_1 = np.array(wp_1)
    wp_2 = np.array(wp_2)
    cube_point = np.array(cube_point)
    vector_1 = wp_2 - wp_1
    vector_2 = cube_point - wp_1
    theta = np.around(np.dot(vector_1, vector_2) / (magnitude(vector_1) *  magnitude(vector_2)), 5)
    print(theta)
    theta = m.degrees(np.arccos(theta))
    
    return theta

def rotate_cube(point: PosVec3, radius, theta):
    x = [point.X - radius, point.X + radius]
    y = [point.Y - radius, point.Y + radius]
    z = [point.Z - radius, point.Z + radius]
    pts = np.stack((x, y, z), 0)
    vertices = (np.array(np.meshgrid(*pts)).T).reshape(2**3,3)
    new_vertices = []
    for i in vertices:
        z_value = i[2]
        a = i.reshape(3,1)
        a[2] = 1
        b = np.matmul(rotate_z(theta, point), a)
        b[2] = z_value
        ax.scatter(b[0], b[1], b[2], color = 'y', marker = '*', s=100)
        new_vertices.append([b[0,0], b[1,0], b[2,0]])

    return new_vertices

def rotate_z(theta, pos: PosVec3):
    theta = np.deg2rad(theta)
    cos_angle = np.around(cos(theta), 5)
    sin_angle = np.around(sin(theta), 5)

    return np.matrix([[cos_angle, -sin_angle, (-pos.X * cos_angle + pos.Y * sin_angle + pos.X)],
                        [sin_angle, cos_angle, (-pos.X * sin_angle - pos.Y * cos_angle + pos.Y)],
                        [0, 0, 1]])

def plot_new_point(point: PosVec3):
    ax.scatter(point.X, point.Y, point.Z, color = 'y', marker = '*', s=100)

def plot_sphere(sphere_pos: PosVec3, radius: float):
    u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    x = (radius) * np.cos(u)*np.sin(v) + sphere_pos.X
    y = (radius) * np.sin(u)*np.sin(v) + sphere_pos.Y
    z = (radius) * np.cos(v) + sphere_pos.Z
    ax.plot_wireframe(x,y,z)

def plot_cube(point: PosVec3, radius, angle):
    theta = np.radians(angle)
    d = [-radius, radius]
    x = [point.X - radius, point.X + radius]
    y = [point.Y - radius, point.Y + radius]
    z = [point.Z - radius, point.Z + radius]
    for s, e in combinations(np.array(list(product(d,d,d))), 2):
        if np.sum(np.abs(s-e)) == d[1] - d[0]:
            s_rotated = [s[0] * cos(theta) - s[1] * sin(theta) + point.X, 
                        s[0] * sin(theta) + s[1] * cos(theta) + point.Y,
                        s[2] + point.Z]
            e_rotated = [e[0] * cos(theta) - e[1] * sin(theta) + point.X, 
                        e[0] * sin(theta) + e[1] * cos(theta) + point.Y,
                        e[2] + point.Z]
            ax.plot3D(*zip(s_rotated,e_rotated), color="g")

def plot_line(pos_1: PosVec3, pos_2: PosVec3, mark):
   ax.plot([pos_1.X, pos_2.X], [pos_1.Y, pos_2.Y], [pos_1.Z, pos_2.Z], color = mark)

def plot_line_new(pos_1: PosVec3, pos_2: PosVec3, new_pos: PosVec3):
   ax.plot([new_pos.X, pos_2.X], [new_pos.Y, pos_2.Y], [new_pos.Z, pos_2.Z], color = 'g')
   ax.plot([pos_1.X, new_pos.X], [pos_1.Y, new_pos.Y], [pos_1.Z, new_pos.Z], color = 'g')

def get_perp(wp1: PosVec3, wp2: PosVec3, obstacle_pos: PosVec3, radius_saftey: float):
    slope = (wp2.Y - wp1.Y) / (wp2.X - wp1.X)
    slope_inv = -1 / slope
    
    a = 1 + m.pow(slope_inv, 2)
    b = -2 * obstacle_pos.X + (2 * (slope_inv) * obstacle_pos.X)
    c = (m.pow(slope_inv, 2) * m.pow(obstacle_pos.X, 2)) + m.pow(obstacle_pos.X, 2) - m.pow(radius_saftey, 2)
    x1 = (-b + m.sqrt(m.pow(b, 2) - 4 * a * c)) / (2 * a)
    x2 = (-b - m.sqrt(m.pow(b, 2) - 4 * a * c)) / (2 * a)
    y1 = (slope_inv) * (x1 - obstacle_pos.X) + obstacle_pos.Y
    y2 = (slope_inv) * (x2 - obstacle_pos.X) + obstacle_pos.Y
    
    return [x1,y1, x2, y2]

def combine_obstacles(obstacles: list):
    for x in range(1, len(obstacles)):
        pos_1 = np.array(obstacles[x][0:3])
        radius_1 = obstacles[x][3]
        pos_2 = np.array(obstacles[x - 1][0:3])
        radius_2 = obstacles[x - 1][3]
        dist = np.linalg.norm(pos_2-pos_1)
        if dist <= radius_1 + radius_2:
            radius = (radius_1 + radius_2 + dist) / 2
            center = pos_1 + (pos_2 - pos_1) * (radius - radius_1) / dist
            sphere = center.tolist()
            sphere.append(radius)
            obstacles[x - 1] = 0
            del obstacles[x]
            obstacles.insert(x, sphere) 
            
def collision_possability(wp_1: PosVec3, wp_2: PosVec3, obstacle_pos: PosVec3, radius_saftey: float):

    a = pow(wp_2.X  - wp_1.X) + pow(wp_2.Y  - wp_1.Y) + pow(wp_2.Z  - wp_1.Z)
    b = 2 * ((wp_2.X  - wp_1.X) * (wp_1.X  - obstacle_pos.X) + (wp_2.Y  - wp_1.Y) * (wp_1.Y  - obstacle_pos.Y) + (wp_2.Z  - wp_1.Z) * (wp_1.Z  - obstacle_pos.Z))
    c = pow(wp_1.X) + pow(wp_1.Y) + pow(wp_1.Z) + pow(obstacle_pos.X) + pow(obstacle_pos.Y) + pow(obstacle_pos.Z)
    c = c - 2 * (wp_1.X * obstacle_pos.X + wp_1.Y * obstacle_pos.Y + wp_1.Z * obstacle_pos.Z) - pow(radius_saftey)

    intersect = pow(b) - (4 * a * c)
    t1 = (-b - m.sqrt(m.fabs(intersect))) / (2.0 * a);
    t2 = (-b + m.sqrt(m.fabs(intersect))) / (2.0 * a);
    if intersect < 0 or t1 > 1 or t1 < 0 or t2 > 1 or t2 < 0:
        return False
    else:
        return True

def get_new_waypoint(wp1:PosVec3, wp2: PosVec3, obstacle_pos: PosVec3, cube_vertices: list, radius_saftey: float):
    cube_pos = PosVec3
    potential_wp = []
    min = m.inf
    for x in cube_vertices:
        cube_pos.X = x[0]
        cube_pos.Y = x[1]
        cube_pos.Z = x[2]
        collision_1 = collision_possability(wp1, cube_pos, obstacle_pos, radius_saftey)
        collision_2 = collision_possability(cube_pos, wp2, obstacle_pos, radius_saftey)
        if collision_1 == False and collision_2 == False:
            potential_wp.append(x)
    potential_wp = np.array(potential_wp)
    way_point1 = [wp1.X, wp1.Y, wp1.Z]
    way_point2 = [wp2.X, wp2.Y, wp2.Z]
    for x in potential_wp:
        dist_1 = np.linalg.norm(x - way_point1)
        dist_2 = np.linalg.norm(way_point2 - x)
        dist = dist_1 + dist_2
        if dist < min: 
            min = dist
            way_point = x  
    
    return way_point

wp_1 = PosVec3()
wp_2 = PosVec3()
new_wp = PosVec3()
obstacle_pos = PosVec3()
way_points = [[1, 1, 1], [5.75, 5.75, 5.75]]
obstacles = [[5, 5, 5, 1]]
#obstacles = [[5, 5, 1, 1], [5, 5, 2, 1], [5, 5, 3, 1]]
#obstacles = [[9, 9, 9, 1], [5, 5, 5, 1], [6, 6, 5, 1], [7, 7, 5, 1], [3, 10, 3, 1]]
combine_obstacles(obstacles)
try:
    while True:
        obstacles.remove(0)
except ValueError:
    pass

ax = plt.axes(projection = '3d')
ax.set_xlim([0,10])
ax.set_ylim([0,10])
ax.set_zlim([0,10])
x = 1
z = len(way_points)

while x < z:
    wp_1.X = way_points[x - 1][0]
    wp_1.Y = way_points[x - 1][1]
    wp_1.Z = way_points[x - 1][2]
    wp_2.X = way_points[x][0]
    wp_2.Y = way_points[x][1]
    wp_2.Z = way_points[x][2]
    already_intersect = False
    for y in range(len(obstacles)):
        obstacle_pos.X = obstacles[y][0]
        obstacle_pos.Y = obstacles[y][1]
        obstacle_pos.Z = obstacles[y][2]
        radius = obstacles[y][3]
        collision = collision_possability(wp_1, wp_2, obstacle_pos, radius)
        if collision == True and already_intersect == False:
            cube_point = closest(way_points[x - 1], obstacle_pos, radius)
            theta = rotation_angle(cube_point[0:2], way_points[x - 1][0:2], way_points[x][0:2])
            cube_points = rotate_cube(obstacle_pos, radius, 0)
            plot_cube(obstacle_pos, radius, 0)
            point = get_new_waypoint(wp_1, wp_2, obstacle_pos, cube_points, radius)
            new_wp.X = point[0]
            new_wp.Y = point[1]
            new_wp.Z = point[2]
            plot_line(wp_1, wp_2, 'r')
            plot_new_point(new_wp)
            plot_line_new(wp_1, wp_2, new_wp)
            way_points.insert(x, point.tolist())
            z += 1
            already_intersect = True
        elif already_intersect == True:
            plot_line(wp_1, wp_2, 'r')
        else:
            plot_line(wp_1, wp_2, 'g')
        plot_sphere(obstacle_pos, radius)
    x+=1
plt.show()