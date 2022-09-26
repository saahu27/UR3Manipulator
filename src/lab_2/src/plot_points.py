import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

file = open("../calib_points2.txt")

calib_x = []
calib_y = []
calib_z = []
real_x = []
real_y = []
real_z = []
count = 1

for line in file.readlines():
    f_list = [float(i) for i in line.split(" ")]
    calib_x.append(f_list[0])
    calib_y.append(f_list[1])
    calib_z.append(f_list[2])
    real_x.append(f_list[3])
    real_y.append(f_list[4])
    real_z.append(f_list[5])
    if count == 50:
        break
    count = count + 1

fig1 = plt.figure()
ax1 = fig1.add_subplot(111, projection='3d')
ax1.scatter(calib_x,calib_y,calib_z,'r')
ax1.scatter(real_x,real_y,real_z,'b')
ax1.set_title("3D Plot")
ax1.set_xlabel("x")
ax1.set_ylabel("y")
ax1.set_zlabel("z")

plt.figure()
plt.scatter(calib_x,calib_y,c='r')
plt.scatter(real_x,real_y,c='b')
plt.title("XY")
plt.xlabel("x")
plt.ylabel("y")

plt.figure()
plt.scatter(calib_y,calib_z,c='r')
plt.scatter(real_y,real_z,c='b')
plt.title("YZ")
plt.xlabel("y")
plt.ylabel("z")

plt.figure()
plt.scatter(calib_z,calib_x,c='r')
plt.scatter(real_z,real_x,c='b')
plt.title("ZX")
plt.xlabel("z")
plt.ylabel("x") 

plt.show()

file.close()