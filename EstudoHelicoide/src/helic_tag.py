import rospy
import math
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np
import time
from matplotlib import pyplot as plt
from tf.transformations import euler_from_matrix
import math
import matplotlib.ticker as ticker
from geometry_msgs.msg import Vector3Stamped,Vector3
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from random import *

rospy.init_node('tag_helicoide_node')




def calculaDifAngular(x1,x2,y1,y2):
	difAngular = math.atan2(x2-x1, y2-y1)
	difAngular = abs(math.degrees(difAngular))
	return (difAngular)


global x_ar, y_ar, n
x_ar = []
y_ar = []

plt.ion()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
n = 1000
theta_max = 0
theta = np.linspace(0, theta_max, n)

x = theta
z =  np.sin(theta)
y =  np.cos(theta)
sc = ax.scatter(x, y, z)


fig.show()


def tag_callback(msg):
    #plt.pause(1)
    global x_ar, y_ar, n
    
    for detections in msg.detections:       
        x_ar.append(detections.pose.pose.pose.position.x)
        y_ar.append(detections.pose.pose.pose.position.y)

    dist = math.sqrt(math.pow((x_ar[1]-x_ar[0]),2) + math.pow((y_ar[1]-y_ar[0]),2))
    

    theta_max = dist *100
    theta = np.linspace(0, theta_max, n)
    x = theta
    z =  np.sin(theta)
    y =  np.cos(theta)
    
    sc._offsets3d = (x, y, z)
    
    plt.xlim(0, theta_max)
    x_ar = []
    y_ar = []
    plt.draw()
   
    #ax.plot(x, y, z, 'b', lw=2)
    #ax.plot((0, theta_max), (0,0), (0,0), color='k', lw=2)
    
    #ax.plot(x, y, 0, color='r', lw=1, alpha=0.5)
    #ax.plot(x, [0]*n, z, color='m', lw=1, alpha=0.5)
   
  
    
    

rospy.wait_for_message("/tag_detections", AprilTagDetectionArray)
rospy.Subscriber('tag_detections', AprilTagDetectionArray, tag_callback)

# Mantém o programa em execução
rospy.spin()