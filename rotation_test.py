from cmath import cos
from re import A
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Rectangle
import matplotlib as mpl
import numpy as np
import math as m
import numpy
from pyrsistent import b
from plot_ssoa import magnitude

#Square Dimensions
diameter = 30
radius = diameter / 2
rect_origin = np.array([-10,10])
rect_center = np.array([-10 + (diameter / 2), 10 + (diameter / 2)])

start_point = [0, 0]      #x values for vector 1
end_point = [0,50]     #y values for vector 1
edge_point = [-10, 10]     #y values for vector 2
vector_1 = [end_point[0] - start_point[0], end_point[1] - start_point[1]]
vector_2 = [edge_point[0] - start_point[0], edge_point[1] - start_point[1]]

theta = (np.dot(vector_1, vector_2) / (magnitude(vector_1) *  magnitude(vector_2)))
theta = m.degrees(np.arccos(theta))

if(vector_1[0] != 0):
    slope = vector_1[1] / vector_1[0]
else:
    slope = 0
slope_square = slope * slope
circle_radius = radius * m.sqrt(2)
radius_square = circle_radius * circle_radius
a = rect_center[0]
e = rect_center[1]
c = start_point[1] - (slope  * start_point[0])
a_square = a * a
e_square = e * e
c_square = c * c
x_val = (radius_square) +(slope_square * radius_square) + (2 * a * slope * e) + (2 * c * e) - (a_square * slope_square) - (2 * a * slope * c) - c_square - e_square
x_val_1 = a - (slope * c) + (slope *e) + m.sqrt(x_val)
x_val_2 = a - (slope * c) + (slope *e) - m.sqrt(x_val)
x_val_1 = x_val_1 / (1 + slope_square)
x_val_2 = x_val_2 / (1 + slope_square)
print(x_val_1)
print(x_val_2)
bx = rect_center[0] - (radius) * np.cos(m.radians(theta)) + (radius) * np.sin(m.radians(theta))
theta = ((-x_val_2 + rect_center[0]) / radius)
theta = (theta * theta) - 1
theta = np.arcsin(-theta) / 2
theta = m.degrees(theta)
print(theta)
by = rect_center[1] - (radius) * np.sin(m.radians(theta)) - (radius) * np.cos(m.radians(theta))
fig = plt.figure(1)
ax = fig.add_subplot()
#First Square 
circle = plt.Circle( (rect_center[0], rect_center[1]), radius * m.sqrt(2), fill = False )
r1 = patches.Rectangle(rect_origin, diameter, diameter, color="blue", alpha=0.50)
#Rotated Square
rec = patches.Rectangle(rect_origin, width=diameter, height=diameter, alpha=0.9,transform=mpl.transforms.Affine2D().rotate_deg_around(*(rect_center[0], rect_center[1]), theta) + ax.transData)
ax.add_patch(rec)
ax.add_patch(r1)
plt.xlim(-40, 40)
plt.ylim(-20, 60)
#plt.axes()
x1 = [start_point[0], end_point[0]]
y1 = [start_point[1], end_point[1]]
x2 = [start_point[0], edge_point[0]]
y2 = [start_point[1], edge_point[1]]
plt.plot(x1, y1, label = "line 1", linestyle="--")
plt.plot(x2, y2, label = "line 2", linestyle="-")
ax.add_artist( circle )
#plt.axis('scaled')
plt.grid(True)
plt.show()

