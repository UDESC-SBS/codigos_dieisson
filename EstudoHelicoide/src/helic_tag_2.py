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
theta_max = 50
theta = np.linspace(0, theta_max, n)

x = theta
z =  np.sin(theta)
y =  np.cos(theta)
sc = ax.scatter(x, y, z) 
sc2 = ax.plot((0, theta_max), (0,0), (0,0), color='k', lw=2)
print(sc2)
#sc3 = ax.scatter(x, y, 0, color='r')  
#sc4 = ax.scatter(x, [0]*n, z, color='m')        
#ax.plot((0, theta_max), (0,0), (0,0), color='k', lw=2)
    
    #ax.plot(x, y, 0, color='r', lw=1, alpha=0.5)
    #ax.plot(x, [0]*n, z, color='m', lw=1, alpha=0.5)
fig.show()  
 
so = [] 
s = []    
    
while not rospy.is_shutdown():
    plt.pause(0.5)
    msg =rospy.wait_for_message("/tag_detections", AprilTagDetectionArray)
    print(len(msg.detections))
    if len(msg.detections) == 2 :
        for detections in msg.detections:   
            x_ar.append(detections.pose.pose.pose.position.x)
            y_ar.append(detections.pose.pose.pose.position.y)
            
        dist = math.sqrt(math.pow((x_ar[1]-x_ar[0]),2) + math.pow((y_ar[1]-y_ar[0]),2))
        
     
        so.append([0,0,0])
        so.append([dist,0,0])
        
        s.append([1,0,0])
        s.append([1,0,0])
        
        
        print(so)
        so = []
        theta_max = dist *100
        theta = np.linspace(0, theta_max, n)
        x = theta
        z =  np.sin(theta)
        y =  np.cos(theta)
        #ax.set_xlim3d((0,theta_max))
        sc._offsets3d = (x, y, z)
        sc2[0].set_data_3d((0, theta_max), (0,0), (0,0))
        fig.canvas.draw_idle()
        

    # sc3._offsets3d =(x, y, 0)  
        #sc4._offsets3d = (x, [0]*n, z)    
        plt.xlim(0, theta_max)

        plt.draw()
        x_ar = []
        y_ar = []

    
