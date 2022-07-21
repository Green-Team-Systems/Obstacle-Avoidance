import matplotlib.pyplot as plt
import numpy as np
# vector, frame_pos, radius, ss_center
def plot_intersect(vector, frame_pos, radius, ss_center):
    

    
    fig = plt.figure()
    ax = plt.axes(projection = '3d')
    ax.set_xlim([-1,50])
    ax.set_ylim([0,50])
    ax.set_zlim([0,50])
    ax.quiver(frame_pos[0],frame_pos[1],frame_pos[2],vector[0],vector[1],vector[2])
    
    
    N = 50
    stride = 5
    u = np.linspace(0, 2 * np.pi, N)
    v = np.linspace(0, np.pi, N)
    x = radius * np.outer(np.cos(u), np.sin(v)) + ss_center[0]
    y = radius * np.outer(np.sin(u),np.sin(v))+ ss_center[1]
    z = radius * np.outer(np.ones(np.size(u)),np.cos(v)) + ss_center[2]
    ax.plot_surface(x,y,z,linewidth= 0.0, cstride = stride, rstride = stride)
    
    
    plt.show()
    

# #drone frame position
# frame_pos = [10,20,20]

# #trajectory to next waypoint
# vector = [20,30,30]

# #sphere center
# ss_center = [10,15,15]
# radius = 5
# plot_intersect(vector, frame_pos, radius, ss_center)