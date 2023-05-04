import rospy
from geometry_msgs.msg import Vector3Stamped
import numpy as np
import time
import matplotlib.pyplot as plt
from tf.transformations import euler_from_matrix
import math
import matplotlib.ticker as ticker

rospy.init_node('acceleration_node')
# Variáveis para armazenar as informações da apriltag
tag_position = np.zeros(3) # posição da apriltag [x, y, z]
tag_velocity = [] # velocidade da apriltag [vx, vy, vz]
tag_last_position = np.zeros(3) # posição anterior da apriltag [x, y, z]
tag_last_velocity = np.zeros(3) # velocidade anterior da apriltag [vx, vy, vz]
tag_time_last_update = rospy.get_rostime() # momento da última atualização da apriltag
tag_acceleration = []
tag_acceleration_list = []
tag_acceleration_list_x = [] # lista para armazenar as acelerações da apriltag
tag_acceleration_list_y = [] # lista para armazenar as acelerações da apriltag
tag_acceleration_list_z = []
tempo = []
tempoi = 0
# Configuração do gráfico
fig, ax = plt.subplots()
line, = ax.plot(tempo,tag_acceleration_list_x,'r-')
line2, = ax.plot(tempo,tag_acceleration_list_y,'g-')
line3, = ax.plot(tempo,tag_acceleration_list_z,'b-')

#line, = ax.plot(1, 1, 'r-')
ax.set_ylim([-40, 40])
ax.yaxis.set_major_locator(ticker.MultipleLocator(5))
x_min = 0
x_max = 10
#ax.xticks(np.arange(min(x_min), max(x_max)+1, 1.0))
ax.set_xlim([x_min, x_max])
ax.xaxis.set_major_locator(ticker.MultipleLocator(1))
#ax.set_xlim(xmin=0)
ax.set_xlabel('Tempo (s)')
ax.set_ylabel('Aceleração (m/s²)')
# Callback para receber as informações da apriltag
#
def tag_callback(msg):
    global tag_acceleration_list_z, x_min,x_max,ax,tag_position, tag_velocity, tag_last_position, tag_last_velocity, tag_time_last_update,tag_acceleration,tag_acceleration_list_x,tag_acceleration_list_y,tempoi,tag_acceleration, tempo
    dt = (rospy.get_rostime() - tag_time_last_update).to_sec()
    
    tag_acceleration_list_y.append(msg.vector.y)
    tag_acceleration_list_x.append(msg.vector.x)
    tag_acceleration_list_z.append(msg.vector.z)

    tag_time_last_update = rospy.get_rostime()

    tempoi = tempoi + dt
    tempo.append(tempoi)
    # Atualiza o gráfico com as novas informações
    line.set_xdata(tempo)
    line.set_ydata(tag_acceleration_list_x)
    line2.set_xdata(tempo)
    line2.set_ydata(tag_acceleration_list_y)
    line3.set_xdata(tempo)
    line3.set_ydata(tag_acceleration_list_z)
    
    fig.canvas.draw_idle()
    x_min += 0.03
    x_max += dt
    ax.set_xlim([x_min, x_max])
    print(dt)
  

 
# Inicializa o nó ROS
# Subscreve no tópico da apriltag
rospy.Subscriber('imu/accel', Vector3Stamped, tag_callback)

plt.show()

# Mantém o programa em execução
rospy.spin()
