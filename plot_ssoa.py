from dis import dis
from matplotlib import projections
import matplotlib.pyplot as plt
import numpy as np
import math as m
from utils.data_classes import PosVec3, MovementCommand, VelVec3

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

def get_new_waypoint_avi(wp: PosVec3, obstacle_pos: PosVec3, radius_saftey: float):
    radius_saftey = radius_saftey
    center = np.array([obstacle_pos.X, obstacle_pos.Y, obstacle_pos.Z])
    way_point = np.array([wp.X, wp.Y, wp.Z])
    dist = np.linalg.norm(way_point - center)
    point = center + (radius_saftey / dist) * (way_point - center)
    return point
    # radius_saftey = radius_saftey * 1.5
    # u = np.linspace(0, np.pi, 20)
    # v = np.linspace(0, 2 * np.pi, 40)
    # x = radius_saftey * np.outer(np.sin(v), np.cos(u)) + obstacle_pos.X
    # y = radius_saftey * np.outer(np.sin(v), np.sin(u)) + obstacle_pos.Y
    # z = radius_saftey * np.outer(np.cos(v), np.ones_like(u)) + obstacle_pos.Z
    # # x = radius_saftey * np.cos(u) * np.sin(v) + obstacle_pos.X
    # # y = radius_saftey * np.sin(u)* np.sin(v) + obstacle_pos.Y
    # # z = radius_saftey * np.cos(v) * np.ones_like(u) + obstacle_pos.Z
    # # x = np.array((1, 2, 3))
    # # y = np.array((1, 2, 3))
    # # z = np.array((1, 2, 3))
    # # print(f'u value: {u}')
    # # print(f'v value: {v}')
    # # print(f'u * v value: {np.sin(u)*np.sin(v)}')
    # a = np.dstack((x, y, z))
    # b = np.array((wp.X, wp.Y, wp.Z))
    # c = b - a
    # dist = np.linalg.norm(c, axis=-1)
    # # print(f' a value: {a}')
    # # print(f' b value: {b}')
    # # print(f' c value: {c}')
    # # print(f' dist value: {dist}')
    # min = np.nanargmin(dist, axis = 1)
    # # print(f' min value: {np.nanmin(dist, axis = 1)}')
    # # print(f' min value index: {min}')
    # location = a[0][min]
    # # print(f' location value: {location[0]}')
    # return location[0]

def get_new_waypoint_josh(wp1, wp2: PosVec3, obstacle_pos: PosVec3, radius_saftey: float):
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
    y1 = m.sqrt(m.pow(radius_saftey, 2) - m.pow(x1 - obstacle_pos.X, 2)) + obstacle_pos.Y
    y2 = m.sqrt(m.pow(radius_saftey, 2) - m.pow(x2 - obstacle_pos.X, 2)) + obstacle_pos.Y
    print("x1 y1", x1, y1)
    print("x2 y2", x2, y2)
    return [x1,y1, x2, y2]

wp_1 = PosVec3()
wp_2 = PosVec3()
new_wp = PosVec3()
obstacle_pos = PosVec3()
way_points = [[1, 1, 1], [10, 10, 3]]
obstacles = [[5, 5, 1, 2]]
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
#for x in range(1, len(way_points)):
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
            point = get_new_waypoint_avi(wp_2, obstacle_pos, radius)
            # point  = get_new_waypoint_josh(wp_1,wp_2, obstacle_pos, radius)
            # slope = (wp_2.Y - wp_1.Y) / (wp_2.X - wp_1.X)
            # slope_inv = -1 / slope
            # t = np.linspace(obstacle_pos.X - 5,obstacle_pos.X + 5,50)
            # y = slope_inv * (t - obstacle_pos.X) + obstacle_pos.Y
            # ax.plot(t, y, 'b', label = 'perpendicular line')
            # ax.scatter(point[0], point[1], 0, color = 'y', marker = '*', s=100)
            # ax.scatter(point[2], point[3], 0, color = 'g', marker = '*', s=100)
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