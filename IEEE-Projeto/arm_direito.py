import rospy
import tf
from visualization_msgs.msg import Marker
import message_filters
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist
import numpy as np
import math
import time
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
## END_SUB_TUTORIAL
from media_pipe_ros1_msg.msg import MediaPipeHumanHandList
from std_msgs.msg import String, Bool
import tinyik
import numpy as np
import math

global x_robot1,z_robot1,x_robot2,z_robot2
x_robot1 = 0
x_robot2 = 0 
z_robot1 = 0 
z_robot2 = 0

global maoEsquerda
global maoDireita
global TrocaBool
maoEsquerda = []
maoDireita = []
TrocaBool = False


def converteX(x):
  return ((x*0.21)/0.5)

def calculaDifAngular(dados):
	#Calcula a diferenca angular
	difAngular = math.atan2(dados[1][0]-dados[0][0], dados[1][1]-dados[0][1])
	difAngular = abs(math.degrees(difAngular))
	#print difAngular
	return (difAngular)
    
def trocaFunc():
	global maoEsquerda, maoDireita,TrocaBool;
	
	if (len(maoEsquerda)>0):
		if (calculaDifAngular([[maoEsquerda[3].x, maoEsquerda[3].y],[maoEsquerda[4].x, maoEsquerda[4].y]])>= 90):
			if (calculaDifAngular([[maoEsquerda[10].x, maoEsquerda[10].y],[maoEsquerda[12].x, maoEsquerda[12].y]]) >= 90):
				if (calculaDifAngular([[maoEsquerda[14].x, maoEsquerda[14].y],[maoEsquerda[16].x, maoEsquerda[16].y]]) >= 100):
					if (calculaDifAngular([[maoEsquerda[18].x, maoEsquerda[18].y],[maoEsquerda[20].x, maoEsquerda[20].y]]) >= 100):	
						if (calculaDifAngular([[maoEsquerda[5].x, maoEsquerda[5].y],[maoEsquerda[8].x, maoEsquerda[8].y]]) >= 100):										
							return True

def move_group_arm(angulos):
  joint_goal = group.get_current_joint_values()  
  joint_goal[0] = 0.0
  joint_goal[1] = (angulos[0]*1.5)/90
  joint_goal[2] = (angulos[1]*1.5)/90
  joint_goal[3] = -1*(joint_goal[2]+joint_goal[1])+1.5
  joint_goal[4] = 0
  group.set_max_velocity_scaling_factor(1.0) 	
  group.set_max_acceleration_scaling_factor(1.0)
  group.set_planning_time(5.0)
  try: 
    group.go(joint_goal, wait=True)
  except:
    return

# def my_callback(msg):			
# 	arm.ee = [((1-maoDireita[8].y)/10.0)*2.0, converteX(x), 0.0]
# 	angulos = np.round(np.rad2deg(arm.angles))
# 	move_group_arm(angulos)

    

rospy.init_node('arm_leo1_direita') 
# moveit start
r1 = moveit_commander
joint_state_topic = ['joint_states:=/leo1/joint_states']
r1.roscpp_initialize(joint_state_topic)  
arm = tinyik.Actuator(['z', [0.105, 0., 0.], 'z', [0.105, 0., 0.]])

robot = r1.RobotCommander(robot_description="leo1/robot_description")  
scene = r1.PlanningSceneInterface(ns="leo1")
group = r1.MoveGroupCommander(robot_description="leo1/robot_description",ns="leo1",name="arm")
while not rospy.is_shutdown():
  msg = rospy.wait_for_message("/mediapipe/human_hand_list", MediaPipeHumanHandList)
  msg_bool = rospy.wait_for_message("/TrocaBool", Bool)
  TrocaBool = msg_bool.data
  my_callback(msg)

#   posicao_msg = rospy.wait_for_message('braco_direita',Marker)    
#   posicao_msg_esquerdo = rospy.wait_for_message('braco_esquerda',Marker)
#   robot_direito_position=rospy.wait_for_message("robot_marker_direito", Marker)
#   robot_esquerdo_position=rospy.wait_for_message("robot_marker_esquerdo", Marker) 
#   callback_robot1(posicao_msg,posicao_msg_esquerdo,robot_direito_position,robot_esquerdo_position)

rospy.spin()


