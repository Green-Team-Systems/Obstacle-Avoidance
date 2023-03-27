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
from sklearn.neighbors import KDTree
from numpy import array, sin, cos
from itertools import product, combinations
import time
from utils.data_classes import PosVec3, MovementCommand, VelVec3
#from ssoa import ClearPathObstacleAvoidance

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
        # ax.scatter(b[0], b[1], b[2], color = 'y', marker = '*', s=100)
        new_vertices.append([b[0,0], b[1,0], b[2,0]])

    return new_vertices

def rotate_z(theta, pos: PosVec3):
    theta = np.deg2rad(theta)
    cos_angle = np.around(cos(theta), 5)
    sin_angle = np.around(sin(theta), 5)

    return np.matrix([[cos_angle, -sin_angle, (-pos.X * cos_angle + pos.Y * sin_angle + pos.X)],
                        [sin_angle, cos_angle, (-pos.X * sin_angle - pos.Y * cos_angle + pos.Y)],
                        [0, 0, 1]])

def plot_new_point(ax, point: PosVec3):
    ax.scatter(point.X, point.Y, point.Z, color = 'y', marker = '*', s=100)

def plot_sphere(ax, sphere_pos: PosVec3, radius: float, col:str):
    u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    x = (radius) * np.cos(u)*np.sin(v) + sphere_pos.X
    y = (radius) * np.sin(u)*np.sin(v) + sphere_pos.Y
    z = (radius) * np.cos(v) + sphere_pos.Z
    
    ax.plot_wireframe(x,y,z, color=col)

def plot_cube(ax, point: PosVec3, radius, angle):
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

def x_y_edge(ax, x_range, y_range, z_range):
    xx, yy = np.meshgrid(x_range, y_range)

    for value in [0, 1]:
        ax.plot_wireframe(xx, yy, z_range[value], color="r")
        ax.plot_surface(xx, yy, z_range[value], color="r", alpha=0.2)


def y_z_edge(ax, x_range, y_range, z_range):
    yy, zz = np.meshgrid(y_range, z_range)

    for value in [0, 1]:
        ax.plot_wireframe(x_range[value], yy, zz, color="r")
        ax.plot_surface(x_range[value], yy, zz, color="r", alpha=0.2)


def x_z_edge(ax, x_range, y_range, z_range):
    xx, zz = np.meshgrid(x_range, z_range)

    for value in [0, 1]:
        ax.plot_wireframe(xx, y_range[value], zz, color="r")
        ax.plot_surface(xx, y_range[value], zz, color="r", alpha=0.2)


def rect_prism(x_range, y_range, z_range, ax):
    x_y_edge(ax, x_range, y_range, z_range)
    y_z_edge(ax, x_range, y_range, z_range)
    x_z_edge(ax, x_range, y_range, z_range)

def plot_line(ax, pos_1: PosVec3, pos_2: PosVec3, mark):
   ax.plot([pos_1.X, pos_2.X], [pos_1.Y, pos_2.Y], [pos_1.Z, pos_2.Z], color = mark)

def plot_line_new(ax, pos_1: PosVec3, pos_2: PosVec3, new_pos: PosVec3):
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

def combine_spheres(obstacles, merge):
    """
    Given the data from KD Tree combines all of the spheres in a certain
    radius into one bigger sphere

    ============= ================================================
    Argument      Description
    ============= ================================================
    obstacles     A list of data that contains the information that
                  was provided by LIDAR. Formatted as a 2d list 
                  with [[x,y,z,radius]....]

    merge         The indexes of the elements inside of obstacles that
                  are in the same radius

    """
    x = 0
    removed = 0
    merge.sort()
    #print(merge)
    while x < len(merge) - 1:
        #print(x)
        index_1 = merge[0]
        index_2 = merge[x + 1] - removed
        #print("I: ", index_1)
        #print("Index: ", index_2)
        pos_1 = np.array(obstacles[index_1][0:3])
        radius_1 = obstacles[index_1][3]
        pos_2 = np.array(obstacles[index_2][0:3])
        radius_2 = obstacles[index_2][3]
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
            #print("dist:", dist)
            #print(pos_1)
            #print(pos_2)
            center = pos_1 + (pos_2 - pos_1) * (radius - radius_1) / dist
        if radius != 0:
            sphere = center.tolist()
            sphere.append(radius)
            del obstacles[index_1]
            del obstacles[index_2 - 1]
            removed += 1
            #print("sphere: ", sphere)
            obstacles.insert(0, sphere)
        x += 1

def combine_obstacles(obstacles):
    """
    method to combine all the obstacles into larger spheres to make 
    calculations faster and easier

    ============= ================================================
    Argument      Description
    ============= ================================================
    obstacles     A list of data that contains the information that
                  was provided by LIDAR. Formatted as a 2d list 
                  with [[x,y,z,radius]....]

    """
    loop = True
    x = 0
    with open('lidardata.txt', 'w') as f:
        print(obstacles, file=f)
        
    while loop == True:
        obstacles_radius = [row[3] for row in obstacles]
        obstacles_location = [row[0:3] for row in obstacles]
        obstacles_location = np.array(obstacles_location)
        #print(X[:1])
        tree = KDTree(obstacles_location, leaf_size = 2)
        #print(obstacles_location)
        ind = tree.query_radius(obstacles_location[:1], r = obstacles_radius[:1])
        combine_spheres(obstacles, ind[0])
        #print(ind[0])
        #print(len(ind[0]))
        #print(x)
        if(len(ind[0]) == 1):
            x += 1
        if(x == len(obstacles)):
            loop = False

def collision_possability(wp_1: PosVec3, wp_2: PosVec3, obstacle_pos: PosVec3, radius_saftey: float):
    """
    Determines if the current trajectory of the drone is on path to hit the object

    ============= ================================================
    Argument      Description
    ============= ================================================
    wp1, wp2      Position vectors of the current position and ending waypoint

    obstacles_pos Position of obstacle

    radius_saftey The radius of the sphere that encloses the obstacle

    """

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

def get_new_waypoint(wp1:PosVec3, wp2: PosVec3, obstacle_pos: PosVec3, cube_vertices: list, radius_saftey: float):
    """
    Gets New Waypoint if it detects that the current path will intersect an object

    ============= ================================================
    Argument      Description
    ============= ================================================
    wp1, wp2      Position vectors of the current position and ending waypoint

    obstacles_pos Position of obstacle

    cube_vertices Different points of the bounding cube around the sphere

    radius_saftey The radius of the sphere that encloses the obstacle

    """
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
    way_point = np.array([wp2.X, wp2.Y, wp2.Z])
    for x in potential_wp:
        dist_1 = np.linalg.norm(x - way_point1)
        dist_2 = np.linalg.norm(way_point2 - x)
        dist = dist_1 + dist_2
        if dist < min: 
            min = dist
            way_point = x
        else: 
            way_point = np.array([wp2.X, wp2.Y, wp2.Z])
    

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

#def on_close(event, object):
#    object.client.simPause(False)

def run_plot(way_points, obstacles, show_plot, self):
    """
    Runs the calculations for finding points around obstacles and displays
    them using matplotlib

    ============= ================================================
    Argument      Description
    ============= ================================================
    way_points    list of points created during trajectory generation

    obstacles     lidar data point cloud
    
    show_plot     boolean that allows for plotting

    """
    wp_1 = PosVec3()
    wp_2 = PosVec3()
    print("Way Point 1:")
    print(wp_1)
    print("Way Point 2:")
    print(wp_2)
    new_wp = PosVec3()
    obstacle_pos = PosVec3()
    # ax = Axes3D(plt.figure())
    # ax = plt.axes(projection = '3d')
    print('plotting started')
    ax = plt.figure(1).add_subplot(projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim([0,70])
    ax.set_ylim([0,70])
    ax.set_zlim([0,70])
    # obstacles = [[5, 5, 1, 1], [5, 5, 2, 1], [5, 5, 3, 1], [7, 7, 7, 1]]
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
                plot_cube(ax, obstacle_pos, radius, theta)
                point = get_new_waypoint(wp_1, wp_2, obstacle_pos, cube_points, radius)
                new_wp.X = point[0]
                new_wp.Y = point[1]
                new_wp.Z = point[2]
                plot_line(ax, wp_1, wp_2, 'r')
                plot_new_point(ax, new_wp)
                plot_line_new(ax, wp_1, wp_2, new_wp)
                way_points.insert(x, point.tolist())
                z += 1
                already_intersect = True
            elif already_intersect == True:
                plot_line(ax, wp_1, wp_2, 'r')
                pass
            else:
                plot_line(ax, wp_1, wp_2, 'b')
                pass
            plot_sphere(ax, obstacle_pos, radius, "lightblue")
        x+=1
    if(show_plot == True):
        self.client.simPause(True)
        plt.show()
        
        self.client.simPause(False)
    
# waypoint = [[10.363398551940918,-11.380178451538086, 5.388492584228516],[70, 0, 5]] 
# # #obstacles = [[5, 5, 1, 1], [5, 5, 2, 1], [5, 5, 3, 1], [7, 7, 7, 1]]
# obstacles = [[23.09999656677246, -8.3700532913208, 0.0, 1], [23.09999656677246, -8.108478546142578, 0.0, 1], [23.09999656677246, -7.848741054534912, 0.0, 1], [23.09999656677246, -7.590784549713135, 0.0, 1], [23.100000381469727, -7.334514617919922, 0.0, 1], [23.09999656677246, -7.079873561859131, 0.0, 1], [23.09999656677246, -6.826793193817139, 0.0, 1], [23.09999656677246, -6.5752129554748535, 0.0, 1], [23.09999656677246, -6.32507848739624, 0.0, 1], [23.100000381469727, -6.076305389404297, 0.0, 1], [23.09999656677246, -5.828843116760254, 0.0, 1], [23.09999656677246, -5.582631587982178, 0.0, 1], [23.100000381469727, -5.337614059448242, 0.0, 1], [23.09999656677246, -5.093730449676514, 0.0, 1], [23.09999656677246, -4.850937843322754, 0.0, 1], [23.09999656677246, -4.609153747558594, 0.0, 1], [23.09999656677246, -4.368338584899902, 0.0, 1], [23.09999656677246, -4.128436088562012, 0.0, 1], [23.09999656677246, -3.8893938064575195, 0.0, 1], [23.09999656677246, -3.651158094406128, 0.0, 1], [23.09999656677246, -3.413689613342285, 0.0, 1], [23.100000381469727, -3.1769134998321533, 0.0, 1], [23.09999656677246, -2.940791606903076, 0.0, 1], [23.09999656677246, -2.7052721977233887, 0.0, 1], [23.09999656677246, -2.470306396484375, 0.0, 1], [23.100000381469727, -2.235856533050537, 0.0, 1], [23.09999656677246, -2.001849889755249, 0.0, 1], [23.09999656677246, -1.7682509422302246, 0.0, 1], [23.303401947021484, -1.548527479171753, 0.0, 1], [23.157033920288086, -1.3052972555160522, 0.0, 1], [23.09999656677246, -1.069417119026184, 0.0, 1], [23.09999656677246, -0.8369796276092529, 0.0, 1], [23.09999656677246, -0.604699969291687, 0.0, 1], [23.100000381469727, -0.37254270911216736, 0.0, 1], [23.09999656677246, -0.14046020805835724, 0.0, 1], [23.09999656677246, 0.09158996492624283, 0.0, 1], [23.100000381469727, 0.32366058230400085, 0.0, 1], [23.09999656677246, 0.555796205997467, 0.0, 1], [23.09999656677246, 0.7880443930625916, 0.0, 1], [23.100000381469727, 1.0204516649246216, 0.0, 1], [23.09999656677246, 1.2530651092529297, 0.0, 1], [23.09999656677246, 1.4859325885772705, 0.0, 1], [23.100000381469727, 1.7190886735916138, 0.0, 1], [23.09999656677246, 1.9526058435440063, 0.0, 1], [23.09999656677246, 2.1865200996398926, 0.0, 1], [23.09999656677246, 2.42087984085083, 0.0, 1], [23.09999656677246, 2.6557328701019287, 0.0, 1], [23.09999656677246, 2.8911290168762207, 0.0, 1], [23.09999656677246, 3.1271183490753174, 0.0, 1], [23.09999656677246, 3.363751173019409, 0.0, 1], [23.100000381469727, 3.60107684135437, 0.0, 1], [23.09999656677246, 3.839146375656128, 0.0, 1], [23.09999656677246, 4.078011989593506, 0.0, 1], [23.100000381469727, 4.317727565765381, 0.0, 1], [23.09999656677246, 4.558344841003418, 0.0, 1], [23.09999656677246, 4.799917221069336, 0.0, 1], [23.09999656677246, 5.0425004959106445, 0.0, 1], [23.09999656677246, 5.286149978637695, 0.0, 1], [23.09999656677246, 5.530921459197998, 0.0, 1], [23.09999656677246, 5.776875019073486, 0.0, 1], [23.09999656677246, 6.024065017700195, 0.0, 1], [23.09999656677246, 6.272557258605957, 0.0, 1], [23.09999656677246, 6.522406578063965, 0.0, 1], [23.100000381469727, 6.773675918579102, 0.0, 1], [23.09999656677246, 7.026431560516357, 0.0, 1], [23.100000381469727, 7.28073787689209, 0.0, 1], [23.100000381469727, 7.536658763885498, 0.0, 1], [23.100000381469727, 7.794262409210205, 0.0, 1], [23.09999656677246, 8.053618431091309, 0.0, 1], [23.202754974365234, 8.35178279876709, 0.0, 1], [23.09999656677246, -8.3700532913208, 0.7149038910865784, 1], [23.100000381469727, -8.108478546142578, 0.7123469710350037, 1], [23.09999656677246, -7.848742485046387, 0.7098795175552368, 1], [23.09999656677246, -7.590784549713135, 0.7075007557868958, 1], [23.09999656677246, -7.334514141082764, 0.705208420753479, 1], [23.09999656677246, -7.07987117767334, 0.7030014991760254, 1], [23.09999656677246, -6.826791763305664, 0.7008790969848633, 1], [23.09999656677246, -6.5752129554748535, 0.6988396644592285, 1], [23.09999656677246, -6.325080394744873, 0.6968823671340942, 1], [23.09999656677246, -6.076303958892822, 0.6950056552886963, 1], [23.09999656677246, -5.828842639923096, 0.6932088136672974, 1], [23.09999656677246, -5.582632541656494, 0.6914910674095154, 1], [23.09999656677246, -5.3376145362854, 0.6898512840270996, 1], [23.100000381469727, -5.093731880187988, 0.6882884502410889, 1], [23.09999656677246, -4.8509368896484375, 0.6868017911911011, 1], [23.09999656677246, -4.60915470123291, 0.6853904724121094, 1], [23.09999656677246, -4.368338584899902, 0.684053897857666, 1], [23.100000381469727, -4.128436088562012, 0.6827914714813232, 1], [23.100000381469727, -3.889394521713257, 0.6816020011901855, 1], [23.09999656677246, -3.651158332824707, 0.6804854869842529, 1], [23.09999656677246, -3.4136898517608643, 0.6794409155845642, 1], [23.100000381469727, -3.1769134998321533, 0.6784679293632507, 1], [23.100000381469727, -2.940791368484497, 0.6775659918785095, 1], [23.09999656677246, -2.7052721977233887, 0.6767348051071167, 1], [23.09999656677246, -2.470306396484375, 0.6759737730026245, 1], [23.09999656677246, -2.2358555793762207, 0.6752822995185852, 1], [23.100000381469727, -2.0018506050109863, 0.6746605634689331, 1], [23.09999656677246, -1.7682509422302246, 0.6741076707839966, 1], [23.303401947021484, -1.548527479171753, 0.6795550584793091, 1], [23.15703010559082, -1.3052972555160522, 0.6748704314231873, 1], [23.09999656677246, -1.0694177150726318, 0.6728611588478088, 1], [23.100000381469727, -0.8369796276092529, 0.6725823283195496, 1], [23.100000381469727, -0.604699969291687, 0.672371506690979, 1], [23.09999656677246, -0.37254270911216736, 0.6722285747528076, 1], [23.100000381469727, -0.14046020805835724, 0.6721537709236145, 1], [23.09999656677246, 0.09158996492624283, 0.6721464395523071, 1], [23.09999656677246, 0.32366058230400085, 0.6722071170806885, 1], [23.09999656677246, 0.5557959079742432, 0.6723357439041138, 1], [23.09999656677246, 0.7880440950393677, 0.6725322008132935, 1], [23.100000381469727, 1.0204516649246216, 0.6727969646453857, 1], [23.09999656677246, 1.2530653476715088, 0.6731294393539429, 1], [23.09999656677246, 1.4859329462051392, 0.6735305190086365, 1], [23.09999656677246, 1.7190885543823242, 0.6739999055862427, 1], [23.100000381469727, 1.952606201171875, 0.674538254737854, 1], [23.100000381469727, 2.1865200996398926, 0.6751455664634705, 1], [23.100000381469727, 2.42087984085083, 0.6758222579956055, 1], [23.09999656677246, 2.6557323932647705, 0.6765686273574829, 1], [23.09999656677246, 2.891129732131958, 0.6773850917816162, 1], [23.100000381469727, 3.127119779586792, 0.6782721281051636, 1], [23.09999656677246, 3.3637514114379883, 0.6792300343513489, 1], [23.09999656677246, 3.6010756492614746, 0.6802592277526855, 1], [23.100000381469727, 3.8391454219818115, 0.6813606023788452, 1], [23.09999656677246, 4.078012466430664, 0.6825346350669861, 1], [23.09999656677246, 4.317726135253906, 0.6837817430496216, 1], [23.09999656677246, 4.558343887329102, 0.6851028203964233, 1], [23.100000381469727, 4.799917221069336, 0.6864981055259705, 1], [23.09999656677246, 5.042500019073486, 0.6879688501358032, 1], [23.09999656677246, 5.286149978637695, 0.6895157098770142, 1], [23.09999656677246, 5.530921459197998, 0.691139280796051, 1], [23.100000381469727, 5.776875019073486, 0.692840576171875, 1], [23.09999656677246, 6.024064064025879, 0.6946204900741577, 1], [23.100000381469727, 6.272557258605957, 0.6964802742004395, 1], [23.09999656677246, 6.52240514755249, 0.6984204053878784, 1], [23.09999656677246, 6.77367639541626, 0.7004427313804626, 1], [23.100000381469727, 7.026432991027832, 0.7025476098060608, 1], [23.09999656677246, 7.280736446380615, 0.7047364711761475, 1], [23.09999656677246, 7.53665828704834, 0.7070105671882629, 1], [23.09999656677246, 7.794262409210205, 0.709371030330658, 1], [23.100000381469727, 8.053618431091309, 0.7118197083473206, 1], [23.2027587890625, 8.351784706115723, 0.7175355553627014, 1], [23.09999656677246, -8.3700532913208, 1.4310195446014404, 1], [23.09999656677246, -8.108478546142578, 1.425900936126709, 1], [23.09999656677246, -7.8487420082092285, 1.4209623336791992, 1], [23.100000381469727, -7.590786457061768, 1.4162005186080933, 1], [23.09999656677246, -7.33451509475708, 1.4116120338439941, 1], [23.100000381469727, -7.07987117767334, 1.4071946144104004, 1], [23.09999656677246, -6.826791763305664, 1.4029459953308105, 1], [23.100000381469727, -6.5752129554748535, 1.3988637924194336, 1], [23.100000381469727, -6.325080394744873, 1.3949458599090576, 1], [23.09999656677246, -6.076304912567139, 1.391189455986023, 1], [23.09999656677246, -5.828842639923096, 1.3875929117202759, 1], [23.09999656677246, -5.582631587982178, 1.384154200553894, 1], [23.09999656677246, -5.337613582611084, 1.3808715343475342, 1], [23.09999656677246, -5.093730926513672, 1.377743124961853, 1], [23.09999656677246, -4.850937366485596, 1.374767541885376, 1], [23.09999656677246, -4.60915470123291, 1.3719427585601807, 1], [23.09999656677246, -4.368338584899902, 1.369267225265503, 1], [23.09999656677246, -4.128436088562012, 1.3667398691177368, 1], [23.100000381469727, -3.8893935680389404, 1.3643591403961182, 1], [23.09999656677246, -3.651158332824707, 1.3621242046356201, 1], [23.09999656677246, -3.413688898086548, 1.3600335121154785, 1], [23.100000381469727, -3.1769137382507324, 1.3580858707427979, 1], [23.09999656677246, -2.9407920837402344, 1.3562805652618408, 1], [23.100000381469727, -2.705272674560547, 1.354616641998291, 1], [23.100000381469727, -2.470306396484375, 1.353093147277832, 1], [23.09999656677246, -2.235856294631958, 1.3517096042633057, 1], [23.09999656677246, -2.00184965133667, 1.350464105606079, 1], [23.100000381469727, -1.7682509422302246, 1.349358081817627, 1], [23.303401947021484, -1.548527479171753, 1.360262155532837, 1], [23.15703010559082, -1.3052972555160522, 1.3508844375610352, 1], [23.09999656677246, -1.069417119026184, 1.34686279296875, 1], [23.09999656677246, -0.8369796276092529, 1.3463046550750732, 1], [23.09999656677246, -0.604699969291687, 1.3458826541900635, 1], [23.09999656677246, -0.37254270911216736, 1.3455970287322998, 1], [23.09999656677246, -0.14046020805835724, 1.3454464673995972, 1], [23.100000381469727, 0.09158996492624283, 1.3454325199127197, 1], [23.09999656677246, 0.32366058230400085, 1.3455538749694824, 1], [23.09999656677246, 0.555796205997467, 1.345811128616333, 1], [23.09999656677246, 0.7880440950393677, 1.3462042808532715, 1], [23.09999656677246, 1.0204516649246216, 1.346733808517456, 1], [23.100000381469727, 1.2530651092529297, 1.3473998308181763, 1], [23.09999656677246, 1.4859325885772705, 1.3482024669647217, 1], [23.09999656677246, 1.7190885543823242, 1.349142074584961, 1], [23.09999656677246, 1.952605962753296, 1.3502198457717896, 1], [23.09999656677246, 2.1865200996398926, 1.3514354228973389, 1], [23.100000381469727, 2.42087984085083, 1.3527899980545044, 1], [23.09999656677246, 2.6557328701019287, 1.3542838096618652, 1], [23.100000381469727, 2.891129970550537, 1.3559186458587646, 1], [23.09999656677246, 3.1271183490753174, 1.3576936721801758, 1], [23.09999656677246, 3.363751173019409, 1.3596112728118896, 1], [23.09999656677246, 3.6010758876800537, 1.3616716861724854, 1], [23.09999656677246, 3.839146375656128, 1.363876461982727, 1], [23.09999656677246, 4.078011512756348, 1.3662261962890625, 1], [23.09999656677246, 4.317727088928223, 1.3687224388122559, 1], [23.09999656677246, 4.558343887329102, 1.3713667392730713, 1], [23.09999656677246, 4.7999162673950195, 1.3741599321365356, 1], [23.100000381469727, 5.042500972747803, 1.3771038055419922, 1], [23.09999656677246, 5.286149978637695, 1.380199909210205, 1], [23.09999656677246, 5.530921459197998, 1.3834497928619385, 1], [23.09999656677246, 5.776875019073486, 1.3868556022644043, 1], [23.100000381469727, 6.024065971374512, 1.3904187679290771, 1], [23.100000381469727, 6.272555828094482, 1.3941410779953003, 1], [23.09999656677246, 6.52240514755249, 1.3980250358581543, 1], [23.100000381469727, 6.77367639541626, 1.4020724296569824, 1], [23.100000381469727, 7.02643346786499, 1.4062857627868652, 1], [23.09999656677246, 7.280736446380615, 1.4106669425964355, 1], [23.09999656677246, 7.53665828704834, 1.4152195453643799, 1], [23.09999656677246, 7.794260025024414, 1.4199445247650146, 1], [23.09999656677246, 8.053618431091309, 1.4248459339141846, 1], [23.202760696411133, 8.351785659790039, 1.4362872838974, 1], [23.09999656677246, -8.3700532913208, 2.149566173553467, 1], [23.09999656677246, -8.108477592468262, 2.141878128051758, 1], [23.09999656677246, -7.8487420082092285, 2.1344590187072754, 1], [23.09999656677246, -7.590784549713135, 2.1273059844970703, 1], [23.09999656677246, -7.334514617919922, 2.1204137802124023, 1], [23.09999656677246, -7.07987117767334, 2.113779067993164, 1], [23.09999656677246, -6.826791763305664, 2.1073966026306152, 1], [23.100000381469727, -6.57521390914917, 2.1012651920318604, 1], [23.100000381469727, -6.325080394744873, 2.095379590988159, 1], [23.100000381469727, -6.076305389404297, 2.0897369384765625, 1], [23.09999656677246, -5.828842639923096, 2.084334373474121, 1], [23.09999656677246, -5.582632541656494, 2.0791687965393066, 1], [23.100000381469727, -5.337615013122559, 2.0742380619049072, 1], [23.09999656677246, -5.09373140335083, 2.0695395469665527, 1], [23.09999656677246, -4.8509368896484375, 2.0650696754455566, 1], [23.09999656677246, -4.60915470123291, 2.060826063156128, 1], [23.09999656677246, -4.368338584899902, 2.056807041168213, 1], [23.09999656677246, -4.12843656539917, 2.053011178970337, 1], [23.09999656677246, -3.8893935680389404, 2.0494349002838135, 1], [23.09999656677246, -3.6511590480804443, 2.0460774898529053, 1], [23.100000381469727, -3.413689613342285, 2.0429368019104004, 1], [23.100000381469727, -3.1769137382507324, 2.040011405944824, 1], [23.100000381469727, -2.940791606903076, 2.0373001098632812, 1], [23.09999656677246, -2.705272674560547, 2.034799814224243, 1], [23.09999656677246, -2.470306396484375, 2.0325112342834473, 1], [23.100000381469727, -2.2358555793762207, 2.0304322242736816, 1], [23.09999656677246, -2.001849889755249, 2.0285630226135254, 1], [23.100000381469727, -1.7682509422302246, 2.0269012451171875, 1], [23.303401947021484, -1.548527479171753, 2.0432801246643066, 1], [23.157028198242188, -1.3052972555160522, 2.0291941165924072, 1], [23.09999656677246, -1.069417119026184, 2.0231523513793945, 1], [23.09999656677246, -0.8369796276092529, 2.0223145484924316, 1], [23.100000381469727, -0.604699969291687, 2.0216803550720215, 1], [23.09999656677246, -0.3725421130657196, 2.0212507247924805, 1], [23.09999656677246, -0.14046020805835724, 2.021026134490967, 1], [23.09999656677246, 0.09158996492624283, 2.0210039615631104, 1], [23.100000381469727, 0.32366058230400085, 2.0211868286132812, 1], [23.09999656677246, 0.555796205997467, 2.021573066711426, 1], [23.09999656677246, 0.7880443930625916, 2.0221638679504395, 1], [23.100000381469727, 1.0204516649246216, 2.0229592323303223, 1]]
# run_plot(waypoint, obstacles, True)
#why wont u update
