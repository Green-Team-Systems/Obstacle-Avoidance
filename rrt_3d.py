# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
import numpy as np
from obstacle_conversion import obstaclesrrt
from src.rrt.rrt import RRT
from src.search_space.search_space import SearchSpace
from src.utilities.plotting import Plot

X_dimensions = np.array([(0,100),(0,100),(0, 100)])
    #[(0, 100), (0, 100), (0, 100)])  # dimensions of Search Space
# obstacles
Obstacles = np.array(obstaclesrrt)

x_init= (20, 5, 55)  # starting location
x_goal = (70, 80, 80)  # goal location

Q = np.array([(2, 4)])  # length of tree edges
r = 1  # length of smallest edge to check for intersection with obstacles
max_samples = 2000 # max number of samples to take before timing out
prc = 0.1 # probability of checking for a connection to goal

# create Search Space
X = SearchSpace(X_dimensions, Obstacles)

# create rrt_search
rrt = RRT(X, Q, x_init, x_goal, max_samples, r, prc)
path = rrt.rrt_search()



# plot
plot = Plot("rrt_3d")
plot.plot_tree(X, rrt.trees)
if path is not None:
    plot.plot_path(X, path)
plot.plot_obstacles(X, Obstacles)
plot.plot_start(X, x_init)
plot.plot_goal(X, x_goal)
plot.draw(auto_open=True)
