
import pandas
import numpy as np


map = np.load('3darr.npy')

rows = 100
cols = 100

obstacles = []

for z in range(0,100):
    for y in range(0,100):
        for x in range(0,100):
            if (map[z][y][x]):
                obstacles.append([x,y,z])

count = 0
temp = 0

obs = []
templist = []
y = 0
for i in range(len(obstacles)):
    if (i != (len(obstacles) - 1)):
        if (obstacles[i+1][0] == ((obstacles[i][0]) + 1)):
            count = count + 1
            templist.append(obstacles[i])
        else:
                if ((obstacles[i-1][0]) + 1) == (obstacles[i][0]):
                    count = count + 1
                    templist.append(obstacles[i])
                    i = i+1
                    if (temp != 0):
                        if (temp != count):
                            preobs = templist[-count:]
                            del templist[-count:]
                            obs.append(templist)
                            templist = []
                            templist = preobs
                    temp = count
                    count = 0
    else:
                if ((obstacles[i-1][0]) + 1) == (obstacles[i]):
                    count = count + 1
                    templist.append(x)
                    
                    if (temp != count):
                        obs.append(templist[:len(templist)-count])
                        obs.append(templist[-count:])
                    else:
                        obs.append = templist


obstaclesrrt = []
for i in obs:
    xs = [x[0] for x in i]
    ys = [y[1] for y in i]
    zs = [z[2] for z in i]

    if (max(xs) > min(xs)) & (max(ys) > min(ys)) & (max(zs) > min(zs)):
        obstaclesrrt.append((min(xs),min(ys),min(zs),max(xs),max(ys),max(zs)))
    else:
        max_xs = max(xs) 
        max_ys = max(ys)
        max_zs = max(zs)
        if (max(xs) == min(xs)):
            max_xs = max(xs) + 1
        if (max(ys) == min(ys)):
            max_ys = min(ys) + 1
        if (max(zs) == min(zs)):
            max_zs = min(zs) + 1
        obstaclesrrt.append((min(xs),min(ys),min(zs), max_xs,max_ys,max_zs))
'''
pd = pandas.DataFrame(obs)
pd.to_csv("obstacles.csv")

'''



