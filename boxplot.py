from threading import local
from turtle import color
from matplotlib import markers
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import math as m

import matplotlib.patches as patches
from pyparsing import alphas
from mpl_toolkits.mplot3d import Axes3D



def boxes():
    ax = plt.axes()
    
    circle1 = plt.Circle((10,10), 10, color = 'r')
    
    r1 = patches.Rectangle((0,0), 20, 20, color="blue", alpha = 0.5)
    r2=  patches.Rectangle((0,0), 20, 20, color="red", alpha = 0.5)
    r3=  patches.Rectangle((0,0), 20, 20, color="yellow", alpha = 0.5)
    r4=  patches.Rectangle((0,0), 20, 20, color="green", alpha = 0.5)
    r5=  patches.Rectangle((0,0), 20, 20, color="purple", alpha = 0.5)
    
    t2= mpl.transforms.Affine2D().rotate_deg_around(10,10,-18) + ax.transData
    r2.set_transform(t2)
    
    t3= mpl.transforms.Affine2D().rotate_deg_around(10,10,-36) + ax.transData
    r3.set_transform(t3)
    
    t4= mpl.transforms.Affine2D().rotate_deg_around(10,10,-54) + ax.transData
    r4.set_transform(t4)
    
    t5= mpl.transforms.Affine2D().rotate_deg_around(10,10,-72) + ax.transData
    r5.set_transform(t5)
    
    ax.add_patch(r1)
    # ax.add_patch(r2)
    # ax.add_patch(r3)
    # ax.add_patch(r4)
    # ax.add_patch(r5)
    ax.add_patch(circle1)
    
    plt.xlim(-20,40)
    plt.ylim(-20,40)
    # plt.grid(True)
    
    plt.show()

def plot_line(pos_1, pos_2, mark):
   ax.plot([pos_1[0], pos_2[0]], [pos_1[1], pos_2[1]], [pos_1[2], pos_2[2]], color = mark)

    
def boxes_3d():
    # ax = plt.axes(projection = '3d')
    # Plot figure
    
    radius = 3
    u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    x = (radius) * np.cos(u)*np.sin(v)
    y = (radius) * np.sin(u)*np.sin(v)
    z = (radius) * np.cos(v)

    
    ax.plot_wireframe(x,y,z)
    
    # # Create axis
    # axes = [5, 5, 5]
    
    # # Create Data
    # data = np.ones(axes)
    
    # # Control Transparency
    # alpha = 0.5
    
    # # Control colour
    # colors = np.empty(axes + [4], dtype=np.float32)
    # colors[:] = [1, 0, 0, alpha]  # red
    
    
    
    # # Voxels is used to customizations of the
    # # sizes, positions and colors.
    # ax.voxels(data, facecolors=colors)
    

    from numpy import sin, cos
    from itertools import product, combinations
    theta = np.radians(45)
    d = [-3,3]
    for s, e in combinations(np.array(list(product(d,d,d))), 2):
        if np.sum(np.abs(s-e)) == d[1]-d[0]:
            s_rotated = [s[0] * cos(theta) - s[1] * sin(theta), 
                        s[0] * sin(theta) + s[1] * cos(theta),
                        s[2]]
            e_rotated = [e[0] * cos(theta) - e[1] * sin(theta), 
                        e[0] * sin(theta) + e[1] * cos(theta),
                        e[2]]      
            ax.plot3D(*zip(s_rotated,e_rotated), color="g")


    theta = np.radians(0)
    d = [-3,3]
    for s, e in combinations(np.array(list(product(d,d,d))), 2):
        if np.sum(np.abs(s-e)) == d[1]-d[0]:
            s_rotated = [s[0] * cos(theta) - s[1] * sin(theta), 
                        s[0] * sin(theta) + s[1] * cos(theta),
                        s[2]]
            e_rotated = [e[0] * cos(theta) - e[1] * sin(theta), 
                        e[0] * sin(theta) + e[1] * cos(theta),
                        e[2]]      
            ax.plot3D(*zip(s_rotated,e_rotated), color="r")

    
    plot_line([4,-4,0],[3,3,3], 'black')
    plot_line([3,3,3],[0,4.25,3], 'black')
    plot_line([-2.7,2.67,-1],[0,4.25,3], 'black')
    plt.show()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
boxes_3d()