import numpy as np
import time
resolution = 0.1
m = 50 * int(1 / resolution)
n = m * m * 30
x,y,z = 1,1,1
ind = x*m*m+ y*m + z
one_d_array = np.ones(n, dtype=np.uint16)
one_d_array[ind]=10
three_d_array = one_d_array.reshape((m,m,30))
t0 = time.time()
for _ in range(1000000):
    c1 = three_d_array[x][y][z]
t1 = time.time()
print(t1-t0)
print("One Dimension Array Size: {}".format(one_d_array.size * one_d_array.itemsize))
print(one_d_array.nbytes)
print(three_d_array.nbytes)
print("Three Dimension Array Size: {}".format(three_d_array.size * three_d_array.itemsize))
t2 = time.time()
for _ in range(1000000):
    # ind = x*m*m+ y*m + z
    c2 = one_d_array[ind]
t3 = time.time()
print(t3-t2) 