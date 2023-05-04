import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
from random import *


plt.ion()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
n = 1000
theta_max = 10
theta = np.linspace(0, theta_max, n)

x = theta
z =  np.sin(theta)
y =  np.cos(theta)
sc = ax.scatter(x, y, z)
fig.show()

for i in range(0, 100):
    plt.pause(0.5)
    theta_max = 10 + i
    theta = np.linspace(0, theta_max, n)
    x = theta
    z =  np.sin(theta)
    y =  np.cos(theta)
    sc._offsets3d = (x, y, z)
    
    plt.xlim(0, theta_max)
    plt.draw()