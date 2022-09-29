from cmath import cos
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Rectangle
import matplotlib as mpl
import numpy as np
import math as m
import numpy
from plot_ssoa import magnitude

#Plotting the rectangle and vectors 
x1 = [0,0]      #x values for vector 1
y1 = [0,10]     #y values for vector 1
vector_1 = [x1[1] - x1[0], y1[1] - y1[0]]
x2 = [0,-10]    #x values for vector 2
y2 = [0,10]     #y values for vector 2
vector_2 = [x2[1] - x2[0], y2[1] - y2[0]]

#Square Dimensions
diameter = 30
rect_origin = np.array([-10,10])
rect_center = np.array([-10 + (diameter / 2), 10 + (diameter / 2)])

#Theta Calculation 
theta = np.around(np.dot(vector_1, vector_2) / (magnitude(vector_1) *  magnitude(vector_2)), 5)
theta = m.degrees(np.arccos(theta))

fig = plt.figure(1)
ax = fig.add_subplot()
#First Square 
r1 = patches.Rectangle(rect_origin, diameter, diameter, color="blue", alpha=0.50)
#Rotated Square
rec = patches.Rectangle(rect_origin, width=diameter, height=diameter, alpha=0.9,transform=mpl.transforms.Affine2D().rotate_deg_around(*(-10,10), theta) + ax.transData)
ax.add_patch(rec)
ax.add_patch(r1)
plt.xlim(-40, 40)
plt.ylim(-20, 60)
#plt.axes()
plt.plot(x1, y1, label = "line 1", linestyle="--")
plt.plot(x2, y2, label = "line 2", linestyle="-")
#plt.axis('scaled')
plt.grid(True)
plt.show()

