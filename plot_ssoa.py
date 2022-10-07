from cmath import nan
from dis import dis
from distutils.log import WARN
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

ax = plt.axes(projection = '3d')
ax.set_xlim([-50,50])
ax.set_ylim([-50,50])
ax.set_zlim([-50,50])

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

def rotation_angle(cube_point, wp_1, wp_2, radius):
    start_point = np.array(wp_1)
    end_point = np.array(wp_2)
    cube_center = np.array(cube_point)
    #edge_point = [cube_center[0] - radius, cube_center[1] - radius]
    vector_1 = end_point - start_point
    #vector_2 = cube_center - edge_point
    circle_radius = radius * m.sqrt(2)
    radius_square = circle_radius * circle_radius
    a = cube_center[0]
    e = cube_center[1]
    a_square = a * a
    e_square = e * e
    if(vector_1[0] == 0):       #Case 1: X value of vector is the same as the starting point
        x_point = start_point[0]
    elif(vector_1[1] == 0):     #Case 2: Y value of vector is the same as the starting point
        print("test")
        y_point = start_point[1]
        y_point_square = y_point * y_point
        print(y_point)
        x_val = m.sqrt(radius_square - y_point_square + (2 * y_point * e) - e_square)
        x_val_1 = x_val + a
        x_val_2 = -x_val + a
        if(m.fabs(x_val_1 - start_point[0]) > m.fabs(x_val_2 - start_point[0])):    #Chooses the closest x_value
            x_point = x_val_2
        else:
            x_point = x_val_1
    else:                       #Case 3: Standard case where the x or y value are not the same as the starting point
        slope = vector_1[1] / vector_1[0]
        slope_square = slope * slope
        c = start_point[1] - (slope  * start_point[0])
        c_square = c * c
        x_val = (radius_square) +(slope_square * radius_square) + (2 * a * slope * e) + (2 * c * e) - (a_square * slope_square) - (2 * a * slope * c) - c_square - e_square
        x_val_1 = a - (slope * c) + (slope *e) + m.sqrt(x_val)
        x_val_2 = a - (slope * c) + (slope *e) - m.sqrt(x_val)
        x_val_1 = x_val_1 / (1 + slope_square)
        x_val_2 = x_val_2 / (1 + slope_square)
        if(m.fabs(x_val_1 - start_point[0]) > m.fabs(x_val_2 - start_point[0])):    #Chooses the closest x value
            x_point = x_val_2
        else:
            x_point = x_val_1

    theta = ((-x_point + cube_center[0]) / radius)
    theta = (theta * theta) - 1
    theta = np.arcsin(-theta) / 2
    theta = m.degrees(theta)
    print("Theta value ")
    print(theta)
    
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

def plot_sphere(sphere_pos: PosVec3, radius: float, col:str):
    u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    x = (radius) * np.cos(u)*np.sin(v) + sphere_pos.X
    y = (radius) * np.sin(u)*np.sin(v) + sphere_pos.Y
    z = (radius) * np.cos(v) + sphere_pos.Z
    
    ax.plot_wireframe(x,y,z, color=col)

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

def x_y_edge(x_range, y_range, z_range):
    xx, yy = np.meshgrid(x_range, y_range)

    for value in [0, 1]:
        ax.plot_wireframe(xx, yy, z_range[value], color="r")
        ax.plot_surface(xx, yy, z_range[value], color="r", alpha=0.2)


def y_z_edge(x_range, y_range, z_range):
    yy, zz = np.meshgrid(y_range, z_range)

    for value in [0, 1]:
        ax.plot_wireframe(x_range[value], yy, zz, color="r")
        ax.plot_surface(x_range[value], yy, zz, color="r", alpha=0.2)


def x_z_edge(x_range, y_range, z_range):
    xx, zz = np.meshgrid(x_range, z_range)

    for value in [0, 1]:
        ax.plot_wireframe(xx, y_range[value], zz, color="r")
        ax.plot_surface(xx, y_range[value], zz, color="r", alpha=0.2)


def rect_prism(x_range, y_range, z_range):
    x_y_edge(x_range, y_range, z_range)
    y_z_edge(x_range, y_range, z_range)
    x_z_edge(x_range, y_range, z_range)

def plot_line(pos_1: PosVec3, pos_2: PosVec3, mark):
   ax.plot([pos_1.X, pos_2.X], [pos_1.Y, pos_2.Y], [pos_1.Z, pos_2.Z], color = mark)

def plot_line_new(pos_1: PosVec3, pos_2: PosVec3, new_pos: PosVec3):
   ax.plot([new_pos.X, pos_2.X], [new_pos.Y, pos_2.Y], [new_pos.Z, pos_2.Z], color = 'b')
   ax.plot([pos_1.X, new_pos.X], [pos_1.Y, new_pos.Y], [pos_1.Z, new_pos.Z], color = 'b')

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
        radius = 0
        if dist + radius_1 < radius_2:
            radius = radius_2
            center = pos_2
        elif dist + radius_2 < radius_1:
            radius = radius_1
            center = pos_1
        elif dist <= radius_1 + radius_2:
            radius = (radius_1 + radius_2 + dist) / 2
            center = pos_1 + (pos_2 - pos_1) * (radius - radius_1) / dist
        if radius != 0:
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
    print(potential_wp)
    way_point1 = [wp1.X, wp1.Y, wp1.Z]
    way_point2 = [wp2.X, wp2.Y, wp2.Z]
    for x in potential_wp:
        dist_1 = np.linalg.norm(x - way_point1)
        dist_2 = np.linalg.norm(way_point2 - x)
        dist = dist_1 + dist_2
        if dist < min: 
            min = dist
            way_point = x  
    #way_point = np.array([20,0,0])
    return way_point

def cuboid_data(center, size):
    """
    Create a data array for cuboid plotting.


    ============= ================================================
    Argument      Description
    ============= ================================================
    center        center of the cuboid, triple
    size          size of the cuboid, triple, (x_length,y_width,z_height)
    :type size: tuple, numpy.array, list
    :param size: size of the cuboid, triple, (x_length,y_width,z_height)
    :type center: tuple, numpy.array, list
    :param center: center of the cuboid, triple, (x,y,z)


    """

    # suppose axis direction: x: to left; y: to inside; z: to upper
    # get the (left, outside, bottom) point
    o = [a - b / 2 for a, b in zip(center, size)]
    # get the length, width, and height
    l, w, h = size
    x = [[o[0], o[0] + l, o[0] + l, o[0], o[0]],  # x coordinate of points in bottom surface
         [o[0], o[0] + l, o[0] + l, o[0], o[0]],  # x coordinate of points in upper surface
         [o[0], o[0] + l, o[0] + l, o[0], o[0]],  # x coordinate of points in outside surface
         [o[0], o[0] + l, o[0] + l, o[0], o[0]]]  # x coordinate of points in inside surface
    y = [[o[1], o[1], o[1] + w, o[1] + w, o[1]],  # y coordinate of points in bottom surface
         [o[1], o[1], o[1] + w, o[1] + w, o[1]],  # y coordinate of points in upper surface
         [o[1], o[1], o[1], o[1], o[1]],          # y coordinate of points in outside surface
         [o[1] + w, o[1] + w, o[1] + w, o[1] + w, o[1] + w]]    # y coordinate of points in inside surface
    z = [[o[2], o[2], o[2], o[2], o[2]],                        # z coordinate of points in bottom surface
         [o[2] + h, o[2] + h, o[2] + h, o[2] + h, o[2] + h],    # z coordinate of points in upper surface
         [o[2], o[2], o[2] + h, o[2] + h, o[2]],                # z coordinate of points in outside surface
         [o[2], o[2], o[2] + h, o[2] + h, o[2]]]                # z coordinate of points in inside surface

    return x, y, z

def run_plot(way_points, obstacles):
    wp_1 = PosVec3()
    wp_2 = PosVec3()
    new_wp = PosVec3()
    obstacle_pos = PosVec3()
    # obstacles = [[5, 5, 1, 1], [5, 5, 2, 1], [5, 5, 3, 1]]
    # obstacles = [[9, 9, 9, 1], [5, 5, 5, 1], [6, 6, 5, 1], [7, 7, 5, 1], [3, 10, 3, 1]]
    x = 1
    z = len(way_points)
    # for obstacle in obstacles:
    #     obstacle_p = PosVec3()
    #     obstacle_p.X = obstacle[0]
    #     obstacle_p.Y = obstacle[1]
    #     obstacle_p.Z = obstacle[2]
        
    #     plot_sphere(obstacle_p, obstacle[3], "r")
    combine_obstacles(obstacles)
    center = [43, 0, 7.5]
    length = 20 * 2
    width = 20 * 2
    height = 7.5 * 2
    X, Y, Z = cuboid_data(center, (length, width, height))
    ax.plot_surface(np.array(X), np.array(Y), np.array(Z), color='b', rstride=1, cstride=1, alpha=0.1)
    try:
        while True:
            obstacles.remove(0)
    except ValueError:
        pass
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
                theta = rotation_angle(obstacles[y][0:2], way_points[x - 1][0:2], way_points[x][0:2], radius)
                cube_points = rotate_cube(obstacle_pos, radius, theta)
                plot_cube(obstacle_pos, radius, theta)
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
                plot_line(wp_1, wp_2, 'b')
            
            plot_sphere(obstacle_pos, radius, "lightblue")
        x+=1
    plt.show()
