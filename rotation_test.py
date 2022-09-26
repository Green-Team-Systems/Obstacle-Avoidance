import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib as mpl
import numpy as np
import math as m
import numpy
from plot_ssoa import magnitude
#Plotting the rectangle and vectors 
#rectangle = plt.Rectangle((-10,10), 50, 20, fc='blue', ec="red")
#plt.gca().add_patch(rectangle)

x1 = [0,0]      #x values for vector 1
y1 = [0,10]     #y values for vector 1
vector_1 = [x1[1] - x1[0], y1[1] - y1[0]]
x2 = [0,-10]    #x values for vector 2
y2 = [0,10]     #y values for vector 2
vector_2 = [x2[1] - x2[0], y2[1] - y2[0]]
diameter = 30
rect_origin = (-10,10)
theta = np.around(np.dot(vector_1, vector_2) / (magnitude(vector_1) *  magnitude(vector_2)), 5)
theta = m.degrees(np.arccos(theta))

fig = plt.figure()
ax = fig.add_subplot(111)

r1 = patches.Rectangle(rect_origin, diameter, diameter, color="blue", alpha=0.50)
r2 = patches.Rectangle(rect_origin, diameter, diameter, color="red",  alpha=0.50)

t2 = mpl.transforms.Affine2D().rotate_deg(theta) ax.transData
r2.set_transform(t2)

ax.add_patch(r1)
ax.add_patch(r2)

plt.xlim(-20, 60)
plt.ylim(-20, 60)

#plt.axes()
plt.plot(x1, y1, label = "line 1", linestyle="--")
plt.plot(x2, y2, label = "line 2", linestyle="-")
#plt.axis('scaled')
plt.grid(True)
plt.show()

