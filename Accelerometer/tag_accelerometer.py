import rospy
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np
import time
import matplotlib.pyplot as plt
from tf.transformations import euler_from_matrix
import math
import matplotlib.ticker as ticker
from geometry_msgs.msg import Vector3Stamped,Vector3

rospy.init_node('tag_acceleration_node')
acl_pub = rospy.Publisher('simu_imu/accel', Vector3Stamped, queue_size=3)
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
def gravitation_from_angle(angle):
    return (angle * 9.8 / 90)
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

# Callback para receber as informações da apriltag
#
def tag_callback(msg):
    global tag_acceleration_list_z, x_min,x_max,ax,tag_position, tag_velocity, tag_last_position, tag_last_velocity, tag_time_last_update,tag_acceleration,tag_acceleration_list_x,tag_acceleration_list_y,tempoi,tag_acceleration, tempo

    # Atualiza a posição da apriltag
    tag_position = [msg.detections[0].pose.pose.pose.position.x, msg.detections[0].pose.pose.pose.position.y, msg.detections[0].pose.pose.pose.position.z]
    print(msg.detections[0].pose.pose.pose.position.z)
    # Calcula o tempo desde a última atualização
    dt = (rospy.get_rostime() - tag_time_last_update).to_sec()

    for i, j in zip(tag_position,tag_last_position): 
        tag_velocity.append((i - j)/ dt)

    # Calcula a velocidade da apriltag
    #tag_velocity = (tag_position - tag_last_position) / dt
    for i, j in zip(tag_velocity,tag_last_velocity): 
        tag_acceleration.append((i - j)/dt)
    # Calcula a aceleração da apriltag
    #tag_acceleration = (tag_velocity - tag_last_velocity) / dt

       # Extrai a matriz de rotação da detecção da apriltag
    rot_mat = msg.detections[0].pose.pose.pose.orientation
        # Converte a matriz de rotação em ângulos de Euler
    euler_angles = euler_from_quaternion(rot_mat.x, rot_mat.y, rot_mat.z, rot_mat.w)    
        # Extrai o ângulo de inclinação da apriltag (ângulo em relação ao sistema de coordenadas fixo)
    incl_angle = euler_angles[0]
        # Imprime o ângulo de inclinação
    #gravitation_from_angle(incl_angle)
    graus_convertidos = ((math.degrees(euler_angles[0]) + 360) % 360) -180

    acx= -1*tag_acceleration[0]+gravitation_from_angle( math.degrees(euler_angles[1]))
    acy= tag_acceleration[1]+ gravitation_from_angle(graus_convertidos)
    acz= tag_acceleration[2] + 9.8 - abs(gravitation_from_angle(graus_convertidos)) - abs(gravitation_from_angle( math.degrees(euler_angles[1])))
    # Atualiza as informações da apriltag para o próximo ciclo
    tag_acceleration_list_y.append(acy)
    tag_acceleration_list_x.append(acx)
    tag_acceleration_list_z.append(acz)

    tag_last_position = tag_position
    tag_last_velocity = tag_velocity
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
    #ax.relim()
    #ax.autoscale(axis='x',tight=None)
    #ax.autoscale_view()
    
    fig.canvas.draw_idle()
    x_min += 0.03
    x_max += dt
    ax.set_xlim([x_min, x_max])
    #plt.pause(0.3)

    #ax.set_xlim([tempoi, tempoi+10])
    tag_velocity = []
    tag_acceleration = []

    acl_msg = Vector3Stamped()
    acl_msg.header.stamp = rospy.get_rostime() 
    acl_msg.header.frame_id = 'accel_simulated'
    acl_msg.vector.x = acx
    acl_msg.vector.y = acy
    acl_msg.vector.z = acz
    acl_pub.publish(acl_msg)

 
# Inicializa o nó ROS
# Subscreve no tópico da apriltag
rospy.Subscriber('tag_detections', AprilTagDetectionArray, tag_callback)

plt.show()

# Mantém o programa em execução
rospy.spin()
