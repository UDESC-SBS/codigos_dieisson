#!/usr/bin/env python
import rospy
import json
import math
import time
from mediapipe_holistic_ros.msg import MediaPipeHolistic
import pandas as pd
import numpy as np
from sklearn.metrics import accuracy_score
from geometry_msgs.msg import Twist
from std_msgs.msg import Time
import skfuzzy as fuzz
from skfuzzy import control as ctrl
#from custom_msg.msg import set_angles,status_arm
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
import sensor_msgs.point_cloud2 as pc2
import tinyik
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn import datasets
from sklearn.decomposition import PCA

import moveit_commander
import moveit_msgs.msg
global yik 
global xik 

r1 = moveit_commander
joint_state_topic = ['joint_states:=/leo1/joint_states']
r1.roscpp_initialize(joint_state_topic) 

robot = r1.RobotCommander(robot_description="leo1/robot_description")  
scene = r1.PlanningSceneInterface(ns="leo1")
group = r1.MoveGroupCommander(robot_description="leo1/robot_description",ns="leo1",name="arm")
#group_gripper = r1.MoveGroupCommander(robot_description="leo1/robot_description",ns="leo1",name="gripper")

xik = 0.05
yik= 0.01

def IK(x,y):
    global yik
    global xik
    arm = tinyik.Actuator(['z', [0.105, 0., 0.], 'z', [0.105, 0., 0.]])
    arm.ee = [y,x,0.0]
    xik = x
    yik = y
    
    angulos = np.round(np.rad2deg(arm.angles))

    # angulos[0] = (angulos[0]%360)*360
    # angulos[1] = (angulos[1]%360)*360
    return angulos

def move_group_arm(angulos):
  #print(angulos)
  joint_goal = group.get_current_joint_values()  
  joint_goal[0] = 0.0
  joint_goal[1] = (angulos[0]*1.5)/90
  joint_goal[2] = (angulos[1]*1.5)/90
  joint_goal[3] =  -1*(joint_goal[2]+joint_goal[1])+1.5
  joint_goal[4] = 0

  group.set_max_velocity_scaling_factor(1.0) 	
  group.set_max_acceleration_scaling_factor(1.0)
  group.set_planning_time(5.0)
  try: 
    group.go(joint_goal, wait=True)
  except:
    return
    

precision=0.005
laserscan = ctrl.Antecedent(np.arange(-1.0, -0.2, precision), 'laserscan')
linear = ctrl.Antecedent(np.arange(-100, 100, precision), 'linear')
angular = ctrl.Antecedent(np.arange(-100, 100, precision), 'angular')


VLinear = ctrl.Consequent(np.arange(-2, 2 , precision), 'VLinear',defuzzify_method='centroid')
VAngular = ctrl.Consequent(np.arange(-2, 2 , precision), 'VAngular',defuzzify_method='centroid')
Ativacao = ctrl.Consequent(np.arange(0, 100 , precision), 'Ativacao',defuzzify_method='centroid')

linear['TrasAltaNegativa'] = fuzz.trapmf(linear.universe, [-100, -100, -95, -55])
linear['TrasMediaNegativa'] = fuzz.trimf(linear.universe, [-100, -50, 0])
linear['Baixa'] = fuzz.trimf(linear.universe, [-50.53, 0, 50])
linear['FrenteMediaPositiva'] = fuzz.trimf(linear.universe, [0, 50, 100])
linear['FrenteAltaPositiva'] = fuzz.trapmf(linear.universe, [55, 95, 100, 100])


angular["EsquerdaMediaNegativa"] = fuzz.trimf(angular.universe, [-100, -50.5, 0])
angular["EsquerdaAltaNegativa"] = fuzz.trapmf(angular.universe, [-100, -100, -95, -55])
angular["Baixa"] = fuzz.trimf(angular.universe, [-50, 0, 50])
angular["DireitaMediaPositiva"] = fuzz.trimf(angular.universe, [0, 50, 100])
angular["DireitaAltaPositiva"] = fuzz.trapmf(angular.universe, [55, 95, 100, 100])


laserscan["Longe"] = fuzz.trapmf(laserscan.universe, [-1.0, -1.0, -0.9, -0.65])
laserscan["Perto"] = fuzz.trimf(laserscan.universe, [-0.9, -0.65, -0.4])
laserscan["MuitoPerto"] = fuzz.trapmf(laserscan.universe, [-0.65, -0.4, -0.2, -0.2])


VLinear["VelocidadeLAltaNegativa"] = fuzz.trapmf(VLinear.universe, [-0.2, -0.2, -0.19, -0.11])
VLinear["VelocidadeLMediaNegativa"] = fuzz.trimf(VLinear.universe, [-0.2, -0.1, 0])
VLinear["VelocidadeLBaixa"] = fuzz.trimf(VLinear.universe, [-0.1, 0, 0.1])
VLinear["VelocidadeLMediaPositiva"] = fuzz.trimf(VLinear.universe, [0, 0.1, 0.2])
VLinear["VelocidadeLAltaPositiva"] = fuzz.trapmf(VLinear.universe, [0.11, 0.19, 0.2, 0.2])


VAngular["VelocidadeAAltaNegativa"] = fuzz.trapmf(VAngular.universe, [-0.2, -0.2, -0.19, -0.11])
VAngular["VelocidadeAMediaNegativa"] = fuzz.trimf(VAngular.universe, [-0.2, -0.1, 0])
VAngular["VelocidadeABaixa"] = fuzz.trimf(VAngular.universe, [-0.1, 0, 0.1])
VAngular["VelocidadeAMediaPositiva"] = fuzz.trimf(VAngular.universe, [0, 0.1, 0.2])
VAngular["VelocidadeAAltaPositiva"] = fuzz.trapmf(VAngular.universe, [0.11, 0.19, 0.2, 0.2])


Ativacao["MuitoPoucoAtivo"] = fuzz.trapmf(Ativacao.universe, [0, 0, 4.167, 37.5])
Ativacao["PoucoAtivo"] = fuzz.trimf(Ativacao.universe, [25, 50, 75])
Ativacao["Ativo"] = fuzz.trapmf(Ativacao.universe, [62.5, 95.83, 100, 100])

regla1 = ctrl.Rule(linear["Baixa"] & angular["Baixa"] & laserscan["Longe"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeABaixa"],Ativacao["MuitoPoucoAtivo"]))

regla2 = ctrl.Rule(linear["Baixa"] & angular["Baixa"] & laserscan["Perto"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeABaixa"],Ativacao["PoucoAtivo"]))

regla3 = ctrl.Rule(linear["Baixa"] & angular["Baixa"] & laserscan["MuitoPerto"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeABaixa"],Ativacao["Ativo"]))

regla4 = ctrl.Rule(linear["Baixa"] & angular["EsquerdaMediaNegativa"] & laserscan["Longe"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeAMediaNegativa"],Ativacao["MuitoPoucoAtivo"]))

regla5 = ctrl.Rule(linear["Baixa"] & angular["EsquerdaMediaNegativa"] & laserscan["Perto"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeAMediaNegativa"],Ativacao["PoucoAtivo"]))

regla6 = ctrl.Rule(linear["Baixa"] & angular["EsquerdaMediaNegativa"] & laserscan["MuitoPerto"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeAMediaNegativa"],Ativacao["Ativo"]))

regla7 = ctrl.Rule(linear["Baixa"] & angular["EsquerdaAltaNegativa"] & laserscan["Longe"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeAAltaNegativa"],Ativacao["MuitoPoucoAtivo"]))

regla8 = ctrl.Rule(linear["Baixa"] & angular["EsquerdaAltaNegativa"] & laserscan["Perto"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeAAltaNegativa"],Ativacao["PoucoAtivo"]))

regla9 = ctrl.Rule(linear["Baixa"] & angular["EsquerdaAltaNegativa"] & laserscan["MuitoPerto"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeAAltaNegativa"],Ativacao["Ativo"]))

regla10 = ctrl.Rule(linear["Baixa"] & angular["DireitaMediaPositiva"] & laserscan["Longe"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeAMediaPositiva"],Ativacao["MuitoPoucoAtivo"]))

regla11 = ctrl.Rule(linear["Baixa"] & angular["DireitaMediaPositiva"] & laserscan["Perto"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeAMediaPositiva"],Ativacao["PoucoAtivo"]))

regla12 = ctrl.Rule(linear["Baixa"] & angular["DireitaMediaPositiva"] & laserscan["MuitoPerto"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeAMediaPositiva"],Ativacao["Ativo"]))

regla13 = ctrl.Rule(linear["Baixa"] & angular["DireitaAltaPositiva"] & laserscan["Longe"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeAAltaPositiva"],Ativacao["MuitoPoucoAtivo"]))

regla14 = ctrl.Rule(linear["Baixa"] & angular["DireitaAltaPositiva"] & laserscan["Perto"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeAAltaPositiva"],Ativacao["PoucoAtivo"]))

regla15 = ctrl.Rule(linear["Baixa"] & angular["DireitaAltaPositiva"] & laserscan["MuitoPerto"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeAAltaPositiva"],Ativacao["Ativo"]))

regla16 = ctrl.Rule(linear["FrenteMediaPositiva"] & angular["Baixa"] & laserscan["Longe"],(VLinear["VelocidadeLMediaPositiva"],VAngular["VelocidadeABaixa"],Ativacao["MuitoPoucoAtivo"]))

regla17 = ctrl.Rule(linear["FrenteMediaPositiva"] & angular["Baixa"] & laserscan["Perto"],(VLinear["VelocidadeLMediaPositiva"],VAngular["VelocidadeABaixa"],Ativacao["PoucoAtivo"]))

regla18 = ctrl.Rule(linear["FrenteMediaPositiva"] & angular["Baixa"] & laserscan["MuitoPerto"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeABaixa"],Ativacao["Ativo"]))

regla19 = ctrl.Rule(linear["FrenteMediaPositiva"] & angular["EsquerdaMediaNegativa"] & laserscan["Longe"],(VLinear["VelocidadeLMediaPositiva"],VAngular["VelocidadeAMediaNegativa"],Ativacao["MuitoPoucoAtivo"]))

regla20 = ctrl.Rule(linear["FrenteMediaPositiva"] & angular["EsquerdaMediaNegativa"] & laserscan["Perto"],(VLinear["VelocidadeLMediaPositiva"],VAngular["VelocidadeAMediaNegativa"],Ativacao["PoucoAtivo"]))

regla21 = ctrl.Rule(linear["FrenteMediaPositiva"] & angular["EsquerdaMediaNegativa"] & laserscan["MuitoPerto"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeAMediaNegativa"],Ativacao["Ativo"]))

regla22 = ctrl.Rule(linear["FrenteMediaPositiva"] & angular["EsquerdaAltaNegativa"] & laserscan["Longe"],(VLinear["VelocidadeLMediaPositiva"],VAngular["VelocidadeAAltaNegativa"],Ativacao["MuitoPoucoAtivo"]))

regla23 = ctrl.Rule(linear["FrenteMediaPositiva"] & angular["EsquerdaAltaNegativa"] & laserscan["Perto"],(VLinear["VelocidadeLMediaPositiva"],VAngular["VelocidadeAAltaNegativa"],Ativacao["PoucoAtivo"]))

regla24 = ctrl.Rule(linear["FrenteMediaPositiva"] & angular["EsquerdaAltaNegativa"] & laserscan["MuitoPerto"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeAAltaNegativa"],Ativacao["Ativo"]))

regla25 = ctrl.Rule(linear["FrenteMediaPositiva"] & angular["DireitaAltaPositiva"] & laserscan["Longe"],(VLinear["VelocidadeLMediaPositiva"],VAngular["VelocidadeAAltaPositiva"],Ativacao["MuitoPoucoAtivo"]))

regla26 = ctrl.Rule(linear["FrenteMediaPositiva"] & angular["DireitaAltaPositiva"] & laserscan["Perto"],(VLinear["VelocidadeLMediaPositiva"],VAngular["VelocidadeAAltaPositiva"],Ativacao["PoucoAtivo"]))

regla27 = ctrl.Rule(linear["FrenteMediaPositiva"] & angular["DireitaAltaPositiva"] & laserscan["MuitoPerto"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeAAltaPositiva"],Ativacao["Ativo"]))

regla28 = ctrl.Rule(linear["FrenteMediaPositiva"] & angular["DireitaMediaPositiva"] & laserscan["Longe"],(VLinear["VelocidadeLMediaPositiva"],VAngular["VelocidadeAMediaPositiva"],Ativacao["MuitoPoucoAtivo"]))

regla29 = ctrl.Rule(linear["FrenteMediaPositiva"] & angular["DireitaMediaPositiva"] & laserscan["Perto"],(VLinear["VelocidadeLMediaPositiva"],VAngular["VelocidadeAMediaPositiva"],Ativacao["PoucoAtivo"]))

regla30 = ctrl.Rule(linear["FrenteMediaPositiva"] & angular["DireitaMediaPositiva"] & laserscan["MuitoPerto"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeAMediaPositiva"],Ativacao["Ativo"]))

regla31 = ctrl.Rule(linear["FrenteAltaPositiva"]  & angular["Baixa"] & laserscan["Longe"],(VLinear["VelocidadeLAltaPositiva"],VAngular["VelocidadeABaixa"],Ativacao["MuitoPoucoAtivo"]))

regla32 = ctrl.Rule(linear["FrenteAltaPositiva"]  & angular["Baixa"] & laserscan["Perto"],(VLinear["VelocidadeLMediaPositiva"],VAngular["VelocidadeABaixa"],Ativacao["PoucoAtivo"]))

regla33 = ctrl.Rule(linear["FrenteAltaPositiva"]  & angular["Baixa"] & laserscan["MuitoPerto"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeABaixa"],Ativacao["Ativo"]))

regla34 = ctrl.Rule(linear["FrenteAltaPositiva"]  & angular["EsquerdaMediaNegativa"] & laserscan["Longe"],(VLinear["VelocidadeLMediaPositiva"],VAngular["VelocidadeAMediaNegativa"],Ativacao["MuitoPoucoAtivo"]))

regla35 = ctrl.Rule(linear["FrenteAltaPositiva"]  & angular["EsquerdaMediaNegativa"] & laserscan["Perto"],(VLinear["VelocidadeLMediaPositiva"],VAngular["VelocidadeAMediaNegativa"],Ativacao["PoucoAtivo"]))

regla36 = ctrl.Rule(linear["FrenteAltaPositiva"]  & angular["EsquerdaMediaNegativa"] & laserscan["MuitoPerto"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeAMediaNegativa"],Ativacao["Ativo"]))

regla37 = ctrl.Rule(linear["FrenteAltaPositiva"]  & angular["EsquerdaAltaNegativa"] & laserscan["Longe"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeAAltaNegativa"],Ativacao["MuitoPoucoAtivo"]))

regla38 = ctrl.Rule(linear["FrenteAltaPositiva"]  & angular["EsquerdaAltaNegativa"] & laserscan["Perto"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeAAltaNegativa"],Ativacao["PoucoAtivo"]))

regla39 = ctrl.Rule(linear["FrenteAltaPositiva"]  & angular["EsquerdaAltaNegativa"] & laserscan["MuitoPerto"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeAAltaNegativa"],Ativacao["Ativo"]))

regla40 = ctrl.Rule(linear["FrenteAltaPositiva"]  & angular["DireitaMediaPositiva"] & laserscan["Longe"],(VLinear["VelocidadeLMediaPositiva"],VAngular["VelocidadeAMediaPositiva"],Ativacao["MuitoPoucoAtivo"]))

regla41 = ctrl.Rule(linear["FrenteAltaPositiva"]  & angular["DireitaMediaPositiva"] & laserscan["Perto"],(VLinear["VelocidadeLMediaPositiva"],VAngular["VelocidadeAMediaPositiva"],Ativacao["PoucoAtivo"]))

regla42 = ctrl.Rule(linear["FrenteAltaPositiva"]  & angular["DireitaMediaPositiva"] & laserscan["MuitoPerto"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeAMediaPositiva"],Ativacao["Ativo"]))

regla43 = ctrl.Rule(linear["FrenteAltaPositiva"]  & angular["DireitaAltaPositiva"] & laserscan["Longe"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeAAltaPositiva"],Ativacao["MuitoPoucoAtivo"]))

regla44 = ctrl.Rule(linear["FrenteAltaPositiva"]  & angular["DireitaAltaPositiva"] & laserscan["Perto"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeAAltaPositiva"],Ativacao["PoucoAtivo"]))

regla45 = ctrl.Rule(linear["FrenteAltaPositiva"]  & angular["DireitaAltaPositiva"] & laserscan["MuitoPerto"],(VLinear["VelocidadeLBaixa"],VAngular["VelocidadeAAltaPositiva"],Ativacao["Ativo"]))

regla46 = ctrl.Rule(linear["TrasMediaNegativa"] & angular["Baixa"] & laserscan["Longe"],(VLinear["VelocidadeLMediaNegativa"],VAngular["VelocidadeABaixa"],Ativacao["MuitoPoucoAtivo"]))

regla47 = ctrl.Rule(linear["TrasMediaNegativa"] & angular["Baixa"] & laserscan["Perto"],(VLinear["VelocidadeLMediaNegativa"],VAngular["VelocidadeABaixa"],Ativacao["PoucoAtivo"]))

regla48 = ctrl.Rule(linear["TrasMediaNegativa"] & angular["Baixa"] & laserscan["MuitoPerto"],(VLinear["VelocidadeLMediaNegativa"],VAngular["VelocidadeABaixa"],Ativacao["Ativo"]))

regla49 = ctrl.Rule(linear["TrasMediaNegativa"] & angular["EsquerdaAltaNegativa"] & laserscan["Longe"],(VLinear["VelocidadeLMediaNegativa"],VAngular["VelocidadeAAltaNegativa"],Ativacao["MuitoPoucoAtivo"]))

regla50 = ctrl.Rule(linear["TrasMediaNegativa"] & angular["EsquerdaAltaNegativa"] & laserscan["Perto"],(VLinear["VelocidadeLMediaNegativa"],VAngular["VelocidadeAAltaNegativa"],Ativacao["PoucoAtivo"]))

regla51 = ctrl.Rule(linear["TrasMediaNegativa"] & angular["EsquerdaAltaNegativa"] & laserscan["MuitoPerto"],(VLinear["VelocidadeLMediaNegativa"],VAngular["VelocidadeAAltaNegativa"],Ativacao["Ativo"]))

regla52 = ctrl.Rule(linear["TrasMediaNegativa"] & angular["EsquerdaMediaNegativa"] & laserscan["Longe"],(VLinear["VelocidadeLMediaNegativa"],VAngular["VelocidadeAMediaNegativa"],Ativacao["MuitoPoucoAtivo"]))

regla53 = ctrl.Rule(linear["TrasMediaNegativa"] & angular["EsquerdaMediaNegativa"] & laserscan["Perto"],(VLinear["VelocidadeLMediaNegativa"],VAngular["VelocidadeAMediaNegativa"],Ativacao["PoucoAtivo"]))

regla54 = ctrl.Rule(linear["TrasMediaNegativa"] & angular["EsquerdaMediaNegativa"] & laserscan["MuitoPerto"],(VLinear["VelocidadeLMediaNegativa"],VAngular["VelocidadeAMediaNegativa"],Ativacao["Ativo"]))

regla55 = ctrl.Rule(linear["TrasMediaNegativa"] & angular["DireitaAltaPositiva"] & laserscan["Longe"],(VLinear["VelocidadeLMediaNegativa"],VAngular["VelocidadeAAltaPositiva"],Ativacao["MuitoPoucoAtivo"]))

regla56 = ctrl.Rule(linear["TrasMediaNegativa"] & angular["DireitaAltaPositiva"] & laserscan["Perto"],(VLinear["VelocidadeLMediaNegativa"],VAngular["VelocidadeAAltaPositiva"],Ativacao["PoucoAtivo"]))

regla57 = ctrl.Rule(linear["TrasMediaNegativa"] & angular["DireitaAltaPositiva"] & laserscan["MuitoPerto"],(VLinear["VelocidadeLMediaNegativa"],VAngular["VelocidadeAAltaPositiva"],Ativacao["Ativo"]))

regla58 = ctrl.Rule(linear["TrasMediaNegativa"] & angular["DireitaMediaPositiva"] & laserscan["Longe"],(VLinear["VelocidadeLMediaNegativa"],VAngular["VelocidadeAMediaPositiva"],Ativacao["MuitoPoucoAtivo"]))

regla59 = ctrl.Rule(linear["TrasMediaNegativa"] & angular["DireitaMediaPositiva"] & laserscan["Perto"],(VLinear["VelocidadeLMediaNegativa"],VAngular["VelocidadeAMediaPositiva"],Ativacao["PoucoAtivo"]))

regla60 = ctrl.Rule(linear["TrasMediaNegativa"] & angular["DireitaMediaPositiva"] & laserscan["MuitoPerto"],(VLinear["VelocidadeLMediaNegativa"],VAngular["VelocidadeAMediaPositiva"],Ativacao["Ativo"]))

regla61 = ctrl.Rule(linear["TrasAltaNegativa"]  & angular["Baixa"] & laserscan["Longe"],(VLinear["VelocidadeLAltaNegativa"],VAngular["VelocidadeABaixa"],Ativacao["MuitoPoucoAtivo"]))

regla62 = ctrl.Rule(linear["TrasAltaNegativa"]  & angular["Baixa"] & laserscan["Perto"],(VLinear["VelocidadeLAltaNegativa"],VAngular["VelocidadeABaixa"],Ativacao["PoucoAtivo"]))

regla63 = ctrl.Rule(linear["TrasAltaNegativa"]  & angular["Baixa"] & laserscan["MuitoPerto"],(VLinear["VelocidadeLAltaNegativa"],VAngular["VelocidadeABaixa"],Ativacao["Ativo"]))

regla64 = ctrl.Rule(linear["TrasAltaNegativa"]  & angular["EsquerdaAltaNegativa"] & laserscan["Longe"],(VLinear["VelocidadeLAltaNegativa"],VAngular["VelocidadeAAltaNegativa"],Ativacao["MuitoPoucoAtivo"]))

regla65 = ctrl.Rule(linear["TrasAltaNegativa"]  & angular["EsquerdaAltaNegativa"] & laserscan["Perto"],(VLinear["VelocidadeLAltaNegativa"],VAngular["VelocidadeAAltaNegativa"],Ativacao["PoucoAtivo"]))

regla66 = ctrl.Rule(linear["TrasAltaNegativa"]  & angular["EsquerdaAltaNegativa"] & laserscan["MuitoPerto"],(VLinear["VelocidadeLAltaNegativa"],VAngular["VelocidadeAAltaNegativa"],Ativacao["Ativo"]))

regla67 = ctrl.Rule(linear["TrasAltaNegativa"]  & angular["EsquerdaMediaNegativa"] & laserscan["Longe"],(VLinear["VelocidadeLAltaNegativa"],VAngular["VelocidadeAMediaNegativa"],Ativacao["MuitoPoucoAtivo"]))

regla68 = ctrl.Rule(linear["TrasAltaNegativa"]  & angular["EsquerdaMediaNegativa"] & laserscan["Perto"],(VLinear["VelocidadeLAltaNegativa"],VAngular["VelocidadeAMediaNegativa"],Ativacao["PoucoAtivo"]))

regla69 = ctrl.Rule(linear["TrasAltaNegativa"]  & angular["EsquerdaMediaNegativa"] & laserscan["MuitoPerto"],(VLinear["VelocidadeLAltaNegativa"],VAngular["VelocidadeAMediaNegativa"],Ativacao["Ativo"]))

regla70 = ctrl.Rule(linear["TrasAltaNegativa"]  & angular["DireitaAltaPositiva"] & laserscan["Longe"],(VLinear["VelocidadeLAltaNegativa"],VAngular["VelocidadeAAltaPositiva"],Ativacao["MuitoPoucoAtivo"]))

regla71 = ctrl.Rule(linear["TrasAltaNegativa"]  & angular["DireitaAltaPositiva"] & laserscan["Perto"],(VLinear["VelocidadeLAltaNegativa"],VAngular["VelocidadeAAltaPositiva"],Ativacao["PoucoAtivo"]))

regla72 = ctrl.Rule(linear["TrasAltaNegativa"]  & angular["DireitaAltaPositiva"] & laserscan["MuitoPerto"],(VLinear["VelocidadeLAltaNegativa"],VAngular["VelocidadeAAltaPositiva"],Ativacao["Ativo"]))

regla73 = ctrl.Rule(linear["TrasAltaNegativa"]  & angular["DireitaMediaPositiva"] & laserscan["Longe"],(VLinear["VelocidadeLAltaNegativa"],VAngular["VelocidadeAMediaPositiva"],Ativacao["MuitoPoucoAtivo"]))

regla74 = ctrl.Rule(linear["TrasAltaNegativa"]  & angular["DireitaMediaPositiva"] & laserscan["Perto"],(VLinear["VelocidadeLAltaNegativa"],VAngular["VelocidadeAMediaPositiva"],Ativacao["PoucoAtivo"]))

regla75 = ctrl.Rule(linear["TrasAltaNegativa"]  & angular["DireitaMediaPositiva"] & laserscan["MuitoPerto"],(VLinear["VelocidadeLAltaNegativa"],VAngular["VelocidadeAMediaPositiva"],Ativacao["Ativo"]))



tipping_ctrl = ctrl.ControlSystem([regla1, regla2, regla3, regla4, regla5, regla6, regla7, regla8, regla9, regla10, regla11, regla12, regla13, regla14, regla15, regla16, regla17, regla18, regla19, regla20, regla21, regla22, regla23, regla24, regla25, regla26, regla27, regla28, regla29, regla30, regla31, regla32, regla33, regla34, regla35, regla36, regla37, regla38, regla39, regla40, regla41, regla42, regla43, regla44, regla45, regla46, regla47, regla48, regla49, regla50, regla51, regla52, regla53, regla54, regla55, regla56, regla57, regla58, regla59, regla60, regla61, regla62, regla63, regla64, regla65, regla66, regla67, regla68, regla69, regla70, regla71, regla72, regla73, regla74, regla75])


#tipping_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5,rule6, rule7, rule8, rule9, rule10, rule11, rule12, rule13,rule14,rule15,rule16, rule17, rule18])

tipping = ctrl.ControlSystemSimulation(tipping_ctrl)
#####print(tipping)

#fuzzy termina aqui


laser_projector = LaserProjection();


global pontoAtivaçãoFrente,MaiorDistanciaFrente,finalFrente,inicioFrente
global leftHandHuman, rightHandHuman,face_mesh_list,pose_list
global json_object,PontosIniciaisME,PontosFinaisME
global ponto, ini, fim, orientacao,membro
global nowRosTime
ponto = []
ini = []
fim = []
membro = []
orientacao = []
def calculaDistanciaEuclidiana(inicial, final):
    a = np.array((inicial[0] ,inicial[1]))
    b = np.array((final[0], final[1]))
    dist = np.linalg.norm(a-b) #distancia euclidiana
    # distanciaEuclidiana = math.sqrt(((dados[0][0]-dados[0][1])**2) + ((dados[1][0] - dados[1][1])**2)  )
    # difAngular = math.atan2(dados[1][0]-dados[0][0], dados[1][1]-dados[0][1])
    # difAngular = (math.degrees(difAngular))
    # difAngular = (difAngular +360)%360 # para ele ir de 0 a 360
    return (dist)
def Norm(min,max, valor):
    return ((valor-min)/(max-min))

def CalcEuclidiana(inicialx,inicialy,finalx,finaly):
    a = np.array((inicialx ,inicialy))
    b = np.array((finalx, finaly))
    dist = np.linalg.norm(a-b)
    
    return dist

def CalculaMaiorAtivação(inicial,final):
    maiordistancia = 0.0
    ponto = 0
    for i in range(0,len(inicial)):
        dist = calculaDistanciaEuclidiana(inicial[i], final[i])
        if dist > maiordistancia:
            maiordistancia = dist
            ponto = i
            maiorinicio = inicial[i]
            maiorfinal = final[i]
    return ponto,maiordistancia,maiorinicio,maiorfinal

def getLidar():
   msg = rospy.wait_for_message("/leo1/scan", LaserScan)
   cloud = laser_projector.projectLaser(msg)
   gen = pc2.read_points(cloud, skip_nans=True, field_names=("x","y"))
   xyz_generator = [];
   for p in gen:
      xyz_generator.append([p[0],p[1]])
   return np.array(xyz_generator);


msg = Twist()
cmd_vel = rospy.Publisher('/leo1/cmd_vel', Twist, queue_size=10);
json_object = []
leftHandHuman = []
rightHandHuman = []
face_mesh_list = []
pose_list = []


dataset = pd.read_csv('hand_dataset.csv')
dataset.head()
teste = dataset.iloc[:, :5].groupby('class')


dataset['class'].value_counts()

ponto = teste.describe()['ponto']['max'].tolist()
orientacao = teste.describe()['orientacao']['max'].tolist() 
ini = teste.describe()['init']['max'].tolist()
fim = teste.describe()['fim']['max'].tolist()

X = dataset.iloc[:, 6:].values


Y = dataset.iloc[:, 4].values

from sklearn.model_selection import train_test_split
X_train, X_test, y_train, y_test = train_test_split(X, Y, test_size=0.33)
from sklearn.preprocessing import StandardScaler
scaler = StandardScaler().fit(X_train)

X_train = scaler.transform(X_train)
X_test = scaler.transform(X_test)

from sklearn.neighbors import KNeighborsClassifier
from sklearn.gaussian_process import GaussianProcessClassifier
from sklearn.svm import SVC
from sklearn.gaussian_process.kernels import RBF

classifier =  SVC(kernel = 'sigmoid',C = 0.5, verbose=True,probability = True)
classifier.fit(X_train, y_train)

y_pred = classifier.predict(X_test)

from sklearn.metrics import classification_report, accuracy_score


import time
def getData():
    global leftHandHuman, rightHandHuman,face_mesh_list,pose_list
    global inicioFrente, finalFrente
    global ponto, ini, fim, orientacao
    global nowRosTime
    leftHandHuman = []
    rightHandHuman = []
    face_mesh_list = []
    pose_list = []
    msg = rospy.wait_for_message("/MediaPipePose/holistic/landmarks", MediaPipeHolistic)

    if msg:    

        nowRosTime=rospy.Time.from_seconds(time.time())
   
    if (msg.left_hand_landmarks):
        for i in range(0,len(msg.left_hand_landmarks)):
            leftHandHuman.append(msg.left_hand_landmarks[i])
                
    if (msg.right_hand_landmarks):
        for i in range(0,len(msg.right_hand_landmarks)):
            rightHandHuman.append(msg.right_hand_landmarks[i])

    if (msg.face_landmarks):
        for i in range(0,len(msg.face_landmarks)):
            face_mesh_list.append(msg.face_landmarks[i])
         
    if (msg.pose_landmarks):
        for i in range(0,len(msg.pose_landmarks)):
            pose_list.append(msg.pose_landmarks[i])

def init():
    import matplotlib.pyplot as plt
    global leftHandHuman, rightHandHuman,PontosIniciaisME,PontosFinaisME,inicioFrente,finalFrente
    global msg
    global ponto, ini, fim, orientacao
    global xik,yik
    valorL = 0
    valorA = 0
    valorU = 0
    valorG = 0
    while not rospy.is_shutdown():
        xyz = getLidar()
        #predicted_prob_arm = 0
        
        getData()
        
      
        pfrente=3
        ptras=4
        pesquerda=2
        pdireita=0
        pupdown = 5
        pgarraopenclose = 1
        afrente = False
        atras = False
        aesquerda = False
        adireita = False
        aupdown = False
        agarraopenclose = False
        AtivaVL = False
        AtivaVA = False
        

        if leftHandHuman:
            ######print("entrou")
            lefthandx = [p.x for p in leftHandHuman]            
            lefthandy = [p.y for p in leftHandHuman]
            ######print(lefthandy[8])
            centroidlefthand = (lefthandx[0], lefthandy[0])
           
            lefthandxc = [p.x - centroidlefthand[0] for p in leftHandHuman]
            lefthandyc = [p.y - centroidlefthand[1] for p in leftHandHuman]
            
            
            coords = list(np.array([[lhh.x - centroidlefthand[0], lhh.y - centroidlefthand[1]] for lhh in leftHandHuman]).flatten())
          
            i = 42+8+10+6+6
            x = 49 
            while i > 0:
                coords.insert(x , 0.0)
                i= i -1
                x = x +1
            
            coords = scaler.transform([coords]) 
            
            
            predicted = classifier.predict(coords)
            predicted_prob = classifier.predict_proba(coords)
            ##print(predicted_prob)

            if predicted_prob[:,pfrente][0] > 0.46:  
                valor =  0              
                centroX = leftHandHuman[int(ponto[pfrente])].x
                centroY = leftHandHuman[int(ponto[pfrente])].y
                if orientacao[pfrente] == 0.0:
                    inicio = centroidlefthand[0] + ini[pfrente] 
                    final =  centroidlefthand[0] + fim[pfrente] 
                    valor = Norm(inicio,final,centroX)
                else:
                    
                    inicio = centroidlefthand[1] + ini[pfrente] 
                    final =  centroidlefthand[1] + fim[pfrente] 
                    valor = Norm(inicio,final,centroY)
                  

                valor = int(valor* 100)
                
                if valor < 0:
                    valor = 0
                if valor > 100:
                    valor = 100

                valorL = valor
                AtivaVL = True
                afrente = True
                atras = False
                aesquerda = False
                adireita = False
                aupdown = False
                agarraopenclose = False
                 
                ####print("aClasse Frente ativa")
            elif predicted_prob[:,ptras][0] > 0.46:
                valor =  0
                centroX = leftHandHuman[int(ponto[ptras])].x
                centroY = leftHandHuman[int(ponto[ptras])].y
                if orientacao[ptras] == 0.0:
                    inicio = centroidlefthand[0] + ini[ptras]
                    final = centroidlefthand[0] + fim[ptras]
                    valor = Norm(inicio,final,centroX)
                else:
                    inicio = centroidlefthand[1] + ini[ptras]
                    final = centroidlefthand[1] + fim[ptras]
                    valor = Norm(inicio,final,centroY)
               
                valor = int(valor* 100)
                
                if valor < 0:
                    valor = 0
                if valor > 100:
                    valor = 100
                valorL = -abs(valor)
                AtivaVL = True
                afrente = False
                atras = True
                aesquerda = False
                adireita = False
                aupdown = False
                agarraopenclose = False
                 
                ####print("Classe Tras ativa")
            elif predicted_prob[:,pesquerda][0] > 0.46:
                valor =  0
                centroX = leftHandHuman[int(ponto[pesquerda])].x
                centroY = leftHandHuman[int(ponto[pesquerda])].y
                ######print(orientacao[pesquerda])
                if orientacao[pesquerda] == 0.0:                    
                    inicio = centroidlefthand[0] + ini[pesquerda]
                    final = centroidlefthand[0] + fim[pesquerda]
                    valor = Norm(inicio,final,centroY)
                  
                else:
                    
                    inicio = centroidlefthand[1] + ini[pesquerda]
                    final = centroidlefthand[1] + fim[pesquerda]
                    valor = Norm(inicio,final,centroY)
               
                valor = int(valor* 100)
                
                if valor < 0:
                    valor = 0
                if valor > 100:
                    valor = 100
                
                valorA = (valor)  
                AtivaVA = True
                afrente = False
                atras = False
                aesquerda = True
                adireita = False
                aupdown = False
                agarraopenclose = False
                            
                ####print("Classe Esquerda ativa")

            elif predicted_prob[:,pdireita][0] > 0.46:
                valor =  0
                centroX = leftHandHuman[int(ponto[pdireita])].x
                centroY = leftHandHuman[int(ponto[pdireita])].y
                if orientacao[pdireita] == 0.0:
                    inicio = centroidlefthand[0] + ini[pdireita]
                    final = centroidlefthand[0] + fim[pdireita]
                    valor = Norm(inicio,final,centroX)
                else:
                    inicio = centroidlefthand[1] + ini[pdireita]
                    final = centroidlefthand[1] + fim[pdireita]
 
                    valor = Norm(inicio,final,centroY)
                   
               
                valor = int(valor* 100)
                
                if valor < 0:
                    valor = 0
                if valor > 100:
                    valor = 100
                valorA = -abs(valor)
                AtivaVA = True
                afrente = False
                atras = False
                aesquerda = False
                adireita = True
                aupdown = False
                agarraopenclose = False
                 
                ####print("Classe Direita ativa")

            elif predicted_prob[:,pupdown][0] > 0.46:
                valor =  0
                centroX = leftHandHuman[int(ponto[pupdown])].x
                centroY = leftHandHuman[int(ponto[pupdown])].y
                if orientacao[pupdown] == 0.0:
                    inicio = centroidlefthand[0] + ini[pupdown]
                    final = centroidlefthand[0] + fim[pupdown]
                    valor = Norm(inicio,final,centroX)
                else:
                    inicio = centroidlefthand[1] + ini[pupdown]
                    final = centroidlefthand[1] + fim[pupdown]
                    valor = Norm(inicio,final,centroY)

               
                valor = int(valor* 100)
                
                if valor < 0:
                    valor = 0
                if valor > 100:
                    valor = 100
                valorU = valor
                afrente = False
                atras = False
                aesquerda = False
                adireita = False
                aupdown = True
                agarraopenclose = False
            
                ####print("Classe UP/DOWN ARM ativa")
            elif predicted_prob[:,pgarraopenclose][0] > 0.46:
                valor =  0
                centroX = leftHandHuman[int(ponto[pgarraopenclose])].x
                centroY = leftHandHuman[int(ponto[pgarraopenclose])].y
                if orientacao[pgarraopenclose] == 0.0:
                    inicio = centroidlefthand[0] + ini[pgarraopenclose]
                    final = centroidlefthand[0] + fim[pgarraopenclose]
                    valor = Norm(inicio,final,centroX)
                else:
                    inicio = centroidlefthand[1] + ini[pgarraopenclose]
                    final = centroidlefthand[1] + fim[pgarraopenclose]
                    valor = Norm(inicio,final,centroY)

               
                valor = int(valor* 100)
                
                if valor < 0:
                    valor = 0
                if valor > 100:
                    valor = 100
                valorGarra = valor
                afrente = False
                atras = False
                aesquerda = False
                adireita = False
                aupdown = False
                agarraopenclose = True
                
        if rightHandHuman:                    
            ######print("entrou")
            righthandx = [p.x for p in rightHandHuman]            
            righthandy = [p.y for p in rightHandHuman]
            ######print(righthandy[8])
            centroidrighthand = (righthandx[0], righthandy[0])
           
            righthandxc = [p.x - centroidrighthand[0] for p in rightHandHuman]
            righthandyc = [p.y - centroidrighthand[1] for p in rightHandHuman]
            
            
            coords = list(np.array([[lhh.x - centroidrighthand[0], lhh.y - centroidrighthand[1]] for lhh in rightHandHuman]).flatten())
          
            i = 42
            x = 6
            while i > 0:
                coords.insert(x , 0.0)
                i= i -1
                x = x +1

            q = 8+10+6+6 #numero de zero depois 
            y = x + 42
            while q > 0:
                coords.insert(y, 0.0)
                q = q -1
                y = y +1
            
            coords = scaler.transform([coords]) 
            
            
            predicted = classifier.predict(coords)
            predicted_prob = classifier.predict_proba(coords)
            ##print(predicted_prob)
            #predicted_prob_arm = predicted_prob[:,pupdown][0] # para verificar se up/down esta ativo
            
            if predicted_prob[:,pfrente][0] > 0.46:  
                valor =  0              
                centroX = rightHandHuman[int(ponto[pfrente])].x
                centroY = rightHandHuman[int(ponto[pfrente])].y
                if orientacao[pfrente] == 0.0:
                    inicio = centroidrighthand[0] + ini[pfrente] 
                    final =  centroidrighthand[0] + fim[pfrente] 
                    valor = Norm(inicio,final,centroX)
                else:
                    
                    inicio = centroidrighthand[1] + ini[pfrente] 
                    final =  centroidrighthand[1] + fim[pfrente] 
                    valor = Norm(inicio,final,centroY)
                  

                valor = int(valor* 100)
                
                if valor < 0:
                    valor = 0
                if valor > 100:
                    valor = 100

                valorL = valor
                AtivaVL = True
                afrente = True
                atras = False
                aesquerda = False
                adireita = False
                aupdown = False
                agarraopenclose = False
                ####print("Classe Frente ativa")
            elif predicted_prob[:,ptras][0] > 0.46:
                valor =  0
                centroX = rightHandHuman[int(ponto[ptras])].x
                centroY = rightHandHuman[int(ponto[ptras])].y
                if orientacao[ptras] == 0.0:
                    inicio = centroidrighthand[0] + ini[ptras]
                    final = centroidrighthand[0] + fim[ptras]
                    valor = Norm(inicio,final,centroX)
                else:
                    inicio = centroidrighthand[1] + ini[ptras]
                    final = centroidrighthand[1] + fim[ptras]
                    valor = Norm(inicio,final,centroY)
               
                valor = int(valor* 100)
                
                if valor < 0:
                    valor = 0
                if valor > 100:
                    valor = 100
                valorL = -abs(valor)
                AtivaVL = True
                afrente = False
                atras = True
                aesquerda = False
                adireita = False
                aupdown = False
                agarraopenclose = False
                ####print("Classe Tras ativa")
            elif predicted_prob[:,pesquerda][0] > 0.46:
                valor =  0
                centroX = rightHandHuman[int(ponto[pesquerda])].x
                centroY = rightHandHuman[int(ponto[pesquerda])].y
                ######print(orientacao[pesquerda])
                if orientacao[pesquerda] == 0.0:                    
                    inicio = centroidrighthand[0] + ini[pesquerda]
                    final = centroidrighthand[0] + fim[pesquerda]
                    valor = Norm(inicio,final,centroY)
                  
                else:
                    
                    inicio = centroidrighthand[1] + ini[pesquerda]
                    final = centroidrighthand[1] + fim[pesquerda]
                    valor = Norm(inicio,final,centroY)
               
                valor = int(valor* 100)
                
                if valor < 0:
                    valor = 0
                if valor > 100:
                    valor = 100
                
                valorA = (valor)  
                AtivaVA = True
                afrente = False
                atras = False
                aesquerda = True
                adireita = False
                aupdown = False 
                agarraopenclose = False          
                ####print("Classe Esquerda ativa")

            elif predicted_prob[:,pdireita][0] > 0.46:
                valor =  0
                centroX = rightHandHuman[int(ponto[pdireita])].x
                centroY = rightHandHuman[int(ponto[pdireita])].y
                if orientacao[pdireita] == 0.0:
                    inicio = centroidrighthand[0] + ini[pdireita]
                    final = centroidrighthand[0] + fim[pdireita]
                    valor = Norm(inicio,final,centroX)
                else:
                    inicio = centroidrighthand[1] + ini[pdireita]
                    final = centroidrighthand[1] + fim[pdireita]
 
                    valor = Norm(inicio,final,centroY)
                   
               
                valor = int(valor* 100)
                
                if valor < 0:
                    valor = 0
                if valor > 100:
                    valor = 100
                valorA = -abs(valor)
                AtivaVA = True
                afrente = False
                atras = False
                aesquerda = False
                adireita = True
                aupdown = False
                agarraopenclose = False
                ####print("Classe Direita ativa")

            elif predicted_prob[:,pupdown][0] > 0.46:
                valor =  0
                centroX = rightHandHuman[int(ponto[pupdown])].x
                centroY = rightHandHuman[int(ponto[pupdown])].y
                if orientacao[pupdown] == 0.0:
                    inicio = centroidrighthand[0] + ini[pupdown]
                    final = centroidrighthand[0] + fim[pupdown]
                    valor = Norm(inicio,final,centroX)
                else:
                    inicio = centroidrighthand[1] + ini[pupdown]
                    final = centroidrighthand[1] + fim[pupdown]
                    valor = Norm(inicio,final,centroY)

               
                valor = int(valor* 100)
                
                if valor < 0:
                    valor = 0
                if valor > 100:
                    valor = 100
                valorU = valor
                afrente = False
                atras = False
                aesquerda = False
                adireita = False
                aupdown = True
                agarraopenclose = False

                ####print("Classe UP/DOWN ARM ativa")
            elif predicted_prob[:,pgarraopenclose][0] > 0.46:
                valor =  0
                centroX = rightHandHuman[int(ponto[pgarraopenclose])].x
                centroY = rightHandHuman[int(ponto[pgarraopenclose])].y
                if orientacao[pgarraopenclose] == 0.0:
                    inicio = centroidrighthand[0] + ini[pgarraopenclose]
                    final = centroidrighthand[0] + fim[pgarraopenclose]
                    valor = Norm(inicio,final,centroX)
                else:
                    inicio = centroidrighthand[1] + ini[pgarraopenclose]
                    final = centroidrighthand[1] + fim[pgarraopenclose]
                    valor = Norm(inicio,final,centroY)

               
                valor = int(valor* 100)
                
                if valor < 0:
                    valor = 0
                if valor > 100:
                    valor = 100
                valorG = valor
                afrente = False
                atras = False
                aesquerda = False
                adireita = False
                aupdown = False
                agarraopenclose = True
            #Boca##########################
        if face_mesh_list:
            boca_list = []
            boca_list.append(face_mesh_list[0])
            boca_list.append(face_mesh_list[17])
            boca_list.append(face_mesh_list[61])
            boca_list.append(face_mesh_list[2]) 


            ######print("entrou")
            bocalistx = [p.x for p in boca_list]            
            bocalisty = [p.y for p in boca_list]
            ######print(righthandy[8])
            centroidbocalist = (bocalistx[0], bocalisty[0])
           
            bocalistxc = [p.x - centroidbocalist[0] for p in boca_list]
            bocalistyc = [p.y - centroidbocalist[1] for p in boca_list]
            
            
            coords = list(np.array([[lhh.x - centroidbocalist[0], lhh.y - centroidbocalist[1]] for lhh in boca_list]).flatten())
          
            i = 42+42
            x = 6
            while i > 0:
                coords.insert(x , 0.0)
                i= i -1
                x = x +1
            q = 10+6+6 #numero de zero depois 
            y = x + 8
            while q > 0:
                coords.insert(y, 0.0)
                q = q -1
                y = y +1
            
            coords = scaler.transform([coords]) 
            
            
            predicted = classifier.predict(coords)
            predicted_prob = classifier.predict_proba(coords)
            #print(predicted_prob)
            #predicted_prob_arm = predicted_prob[:,pupdown][0] # para verificar se up/down esta ativo
            
            if predicted_prob[:,pfrente][0] > 0.46:  
                valor =  0              
                centroX = face_mesh_list[int(ponto[pfrente])].x
                centroY = face_mesh_list[int(ponto[pfrente])].y
                if orientacao[pfrente] == 0.0:
                    inicio = centroidbocalist[0] + ini[pfrente] 
                    final =  centroidbocalist[0] + fim[pfrente] 
                    valor = Norm(inicio,final,centroX)
                else:
                    
                    inicio = centroidbocalist[1] + ini[pfrente] 
                    final =  centroidbocalist[1] + fim[pfrente] 
                    valor = Norm(inicio,final,centroY)
                  

                valor = int(valor* 100)
                
                if valor < 0:
                    valor = 0
                if valor > 100:
                    valor = 100

                valorL = valor
                AtivaVL = True
                afrente = True
                atras = False
                aesquerda = False
                adireita = False
                aupdown = False
                agarraopenclose = False
                ####print("Classe Frente ativa")
            elif predicted_prob[:,ptras][0] > 0.46:
                valor =  0
                centroX = face_mesh_list[int(ponto[ptras])].x
                centroY = face_mesh_list[int(ponto[ptras])].y
                if orientacao[ptras] == 0.0:
                    inicio = centroidbocalist[0] + ini[ptras]
                    final = centroidbocalist[0] + fim[ptras]
                    valor = Norm(inicio,final,centroX)
                else:
                    inicio = centroidbocalist[1] + ini[ptras]
                    final = centroidbocalist[1] + fim[ptras]
                    valor = Norm(inicio,final,centroY)
               
                valor = int(valor* 100)
                
                if valor < 0:
                    valor = 0
                if valor > 100:
                    valor = 100
                valorL = -abs(valor)
                AtivaVL = True
                afrente = False
                atras = True
                aesquerda = False
                adireita = False
                aupdown = False
                agarraopenclose = False
                ####print("Classe Tras ativa")
            elif predicted_prob[:,pesquerda][0] > 0.46:
                valor =  0
                centroX = face_mesh_list[int(ponto[pesquerda])].x
                centroY = face_mesh_list[int(ponto[pesquerda])].y
                ######print(orientacao[pesquerda])
                if orientacao[pesquerda] == 0.0:                    
                    inicio = centroidbocalist[0] + ini[pesquerda]
                    final = centroidbocalist[0] + fim[pesquerda]
                    valor = Norm(inicio,final,centroY)
                  
                else:
                    
                    inicio = centroidbocalist[1] + ini[pesquerda]
                    final = centroidbocalist[1] + fim[pesquerda]
                    valor = Norm(inicio,final,centroY)
               
                valor = int(valor* 100)
                
                if valor < 0:
                    valor = 0
                if valor > 100:
                    valor = 100
                
                valorA = (valor)  
                AtivaVA = True
                afrente = False
                atras = False
                aesquerda = True
                adireita = False
                aupdown = False 
                agarraopenclose = False          
                ####print("Classe Esquerda ativa")

            elif predicted_prob[:,pdireita][0] > 0.46:
                valor =  0
                centroX = face_mesh_list[int(ponto[pdireita])].x
                centroY = face_mesh_list[int(ponto[pdireita])].y
                if orientacao[pdireita] == 0.0:
                    inicio = centroidbocalist[0] + ini[pdireita]
                    final = centroidbocalist[0] + fim[pdireita]
                    valor = Norm(inicio,final,centroX)
                else:
                    inicio = centroidbocalist[1] + ini[pdireita]
                    final = centroidbocalist[1] + fim[pdireita]
 
                    valor = Norm(inicio,final,centroY)
                   
               
                valor = int(valor* 100)
                
                if valor < 0:
                    valor = 0
                if valor > 100:
                    valor = 100
                valorA = -abs(valor)
                AtivaVA = True
                afrente = False
                atras = False
                aesquerda = False
                adireita = True
                aupdown = False
                agarraopenclose = False
                ####print("Classe Direita ativa")

            elif predicted_prob[:,pupdown][0] > 0.46:
                valor =  0
                centroX = face_mesh_list[int(ponto[pupdown])].x
                centroY = face_mesh_list[int(ponto[pupdown])].y
                if orientacao[pupdown] == 0.0:
                    inicio = centroidbocalist[0] + ini[pupdown]
                    final = centroidbocalist[0] + fim[pupdown]
                    valor = Norm(inicio,final,centroX)
                else:
                    inicio = centroidbocalist[1] + ini[pupdown]
                    final = centroidbocalist[1] + fim[pupdown]
                    valor = Norm(inicio,final,centroY)

               
                valor = int(valor* 100)
                
                if valor < 0:
                    valor = 0
                if valor > 100:
                    valor = 100
                valorU = valor
                afrente = False
                atras = False
                aesquerda = False
                adireita = False
                aupdown = True
                agarraopenclose = False

                ####print("Classe UP/DOWN ARM ativa")
            elif predicted_prob[:,pgarraopenclose][0] > 0.46:
                valor =  0
                centroX = face_mesh_list[int(ponto[pgarraopenclose])].x
                centroY = face_mesh_list[int(ponto[pgarraopenclose])].y
                if orientacao[pgarraopenclose] == 0.0:
                    inicio = centroidbocalist[0] + ini[pgarraopenclose]
                    final = centroidbocalist[0] + fim[pgarraopenclose]
                    valor = Norm(inicio,final,centroX)
                else:
                    inicio = centroidbocalist[1] + ini[pgarraopenclose]
                    final = centroidbocalist[1] + fim[pgarraopenclose]
                    valor = Norm(inicio,final,centroY)

               
                valor = int(valor* 100)
                
                if valor < 0:
                    valor = 0
                if valor > 100:
                    valor = 100
                valorG = valor
                afrente = False
                atras = False
                aesquerda = False
                adireita = False
                aupdown = False
                agarraopenclose = True
                ###print("Classe Garra ativa")
            ####
            #rosto##########################
        if face_mesh_list:
            if pose_list:
                rosto_list = []
                rosto_list.append(face_mesh_list[0])
                rosto_list.append(face_mesh_list[152])
                rosto_list.append(face_mesh_list[234])
                rosto_list.append(face_mesh_list[10])
                rosto_list.append(face_mesh_list[454])


                ######print("entrou")
                rosto_listx= [p.x for p in rosto_list]
                rosto_listy= [p.y for p in rosto_list]
                centx = (pose_list[12].x + pose_list[11].x) /2
                centy = (pose_list[12].y + pose_list[11].y) /2
    
                ######print(righthandy[8])
                centroidrostolist = (centx,centy)
            
                # rostolistxc = [p.x - centroidrostolist[0] for p in rosto_listx]
                # rostolistyc = [p.y - centroidrostolist[1] for p in rosto_listx]
                
                
                coords = list(np.array([[lhh.x - centroidrostolist[0], lhh.y - centroidrostolist[1]] for lhh in rosto_list]).flatten())
            
                i = 42+42+8
                x = 6
                while i > 0:
                    coords.insert(x , 0.0)
                    i= i -1
                    x = x +1
                q = 6 + 6 #numero de zero depois 
                y = x + 10
                while q > 0:
                    coords.insert(y, 0.0)
                    q = q -1
                    y = y +1
                
                coords = scaler.transform([coords]) 
                
                ###print(rosto_list)
                ###print
                predicted = classifier.predict(coords)
                predicted_prob = classifier.predict_proba(coords)
                ##print(predicted_prob)
                #predicted_prob_arm = predicted_prob[:,pupdown][0] # para verificar se up/down esta ativo
                if predicted_prob[:,pfrente][0] > 0.46:  
                    valor =  0              
                    centroX = face_mesh_list[int(ponto[pfrente])].x
                    centroY = face_mesh_list[int(ponto[pfrente])].y
                    if orientacao[pfrente] == 0.0:
                        inicio = centroidrostolist[0] + ini[pfrente] 
                        final =  centroidrostolist[0] + fim[pfrente] 
                        valor = Norm(inicio,final,centroX)
                    else:
                        
                        inicio = centroidrostolist[1] + ini[pfrente] 
                        final =  centroidrostolist[1] + fim[pfrente] 
                        valor = Norm(inicio,final,centroY)
                    

                    valor = int(valor* 100)
                    
                    if valor < 0:
                        valor = 0
                    if valor > 100:
                        valor = 100

                    valorL = valor
                    AtivaVL = True
                    afrente = True
                    atras = False
                    aesquerda = False
                    adireita = False
                    aupdown = False
                    agarraopenclose = False
                
                    ####print("Classe Frente ativa")
                elif predicted_prob[:,ptras][0] > 0.46:
                    valor =  0
                    centroX = face_mesh_list[int(ponto[ptras])].x
                    centroY = face_mesh_list[int(ponto[ptras])].y
                    if orientacao[ptras] == 0.0:
                        inicio = centroidrostolist[0] + ini[ptras]
                        final = centroidrostolist[0] + fim[ptras]
                        valor = Norm(inicio,final,centroX)
                    else:
                        inicio = centroidrostolist[1] + ini[ptras]
                        final = centroidrostolist[1] + fim[ptras]
                        valor = Norm(inicio,final,centroY)
                
                    valor = int(valor* 100)
                    
                    if valor < 0:
                        valor = 0
                    if valor > 100:
                        valor = 100
                    valorL = -abs(valor)
                    AtivaVL = True
                    afrente = False
                    atras = True
                    aesquerda = False
                    adireita = False
                    aupdown = False
                    agarraopenclose = False
                    ####print("Classe Tras ativa")
                elif predicted_prob[:,pesquerda][0] > 0.46:
                    valor =  0
                    centroX = face_mesh_list[int(ponto[pesquerda])].x
                    centroY = face_mesh_list[int(ponto[pesquerda])].y
                    ######print(orientacao[pesquerda])
                    if orientacao[pesquerda] == 0.0:                    
                        inicio = centroidrostolist[0] + ini[pesquerda]
                        final = centroidrostolist[0] + fim[pesquerda]
                        valor = Norm(inicio,final,centroY)
                    
                    else:
                        
                        inicio = centroidrostolist[1] + ini[pesquerda]
                        final = centroidrostolist[1] + fim[pesquerda]
                        valor = Norm(inicio,final,centroY)
                
                    valor = int(valor* 100)
                    
                    if valor < 0:
                        valor = 0
                    if valor > 100:
                        valor = 100
                    
                    valorA = (valor) 
                    AtivaVA = True 
                    afrente = False
                    atras = False
                    aesquerda = True
                    adireita = False
                    aupdown = False 
                    agarraopenclose = False          
                    ####print("Classe Esquerda ativa")

                elif predicted_prob[:,pdireita][0] > 0.46:
                    
                    valor =  0
                    centroX = face_mesh_list[int(ponto[pdireita])].x
                    centroY = face_mesh_list[int(ponto[pdireita])].y
                    if orientacao[pdireita] == 0.0:
                        inicio = centroidrostolist[0] + ini[pdireita]
                        final = centroidrostolist[0] + fim[pdireita]
                        valor = Norm(inicio,final,centroX)
                    else:
                        inicio = centroidrostolist[1] + ini[pdireita]
                        final = centroidrostolist[1] + fim[pdireita]
    
                        valor = Norm(inicio,final,centroY)
                    
                
                    valor = int(valor* 100)
                    
                    if valor < 0:
                        valor = 0
                    if valor > 100:
                        valor = 100
                    valorA = -abs(valor)
                    AtivaVA = True
                    afrente = False
                    atras = False
                    aesquerda = False
                    adireita = True
                    aupdown = False
                    agarraopenclose = False
                    ####print("Classe Direita ativa")

                elif predicted_prob[:,pupdown][0] > 0.46:
                    valor =  0
                    centroX = face_mesh_list[int(ponto[pupdown])].x
                    centroY = face_mesh_list[int(ponto[pupdown])].y
                    if orientacao[pupdown] == 0.0:
                        inicio = centroidrostolist[0] + ini[pupdown]
                        final = centroidrostolist[0] + fim[pupdown]
                        valor = Norm(inicio,final,centroX)
                    else:
                        inicio = centroidrostolist[1] + ini[pupdown]
                        final = centroidrostolist[1] + fim[pupdown]
                        valor = Norm(inicio,final,centroY)

                
                    valor = int(valor* 100)
                    
                    if valor < 0:
                        valor = 0
                    if valor > 100:
                        valor = 100
                    valorU = valor
                    afrente = False
                    atras = False
                    aesquerda = False
                    adireita = False
                    aupdown = True
                    agarraopenclose = False

                    ####print("Classe UP/DOWN ARM ativa")
                elif predicted_prob[:,pgarraopenclose][0] > 0.46:
                    valor =  0
                    centroX = face_mesh_list[int(ponto[pgarraopenclose])].x
                    centroY = face_mesh_list[int(ponto[pgarraopenclose])].y
                    if orientacao[pgarraopenclose] == 0.0:
                        inicio = centroidrostolist[0] + ini[pgarraopenclose]
                        final = centroidrostolist[0] + fim[pgarraopenclose]
                        valor = Norm(inicio,final,centroX)
                    else:
                        inicio = centroidrostolist[1] + ini[pgarraopenclose]
                        final = centroidrostolist[1] + fim[pgarraopenclose]
                        valor = Norm(inicio,final,centroY)

                
                    valor = int(valor* 100)
                    
                    if valor < 0:
                        valor = 0
                    if valor > 100:
                        valor = 100
                    valorG = valor
                    afrente = False
                    atras = False
                    aesquerda = False
                    adireita = False
                    aupdown = False
                    agarraopenclose = True
                   
        elif rightHandHuman==[] and leftHandHuman==[] and face_mesh_list == []:
            valorL = 0
            valorA = 0
            valorU = 0
            valorG = 0
        
        if AtivaVL == False :
            valorL =0
        if AtivaVA == False  :
            valorA =0

        menor = 1
        y = 0
        for x in xyz:
            if (x[0] > 0.0):
                if (x[0] < menor):
                    menor = x[0]
                    y = x[1]
                    

        
        #menor = menor * 100
        print(menor)
        # if menor < 0.3:
        #     menor = 0.3
        #print(valorL)
        #print(-abs(menor))
        tipping.input['laserscan'] = -abs(menor)
        tipping.input['linear'] = valorL
        tipping.input['angular'] = valorA
        tipping.compute()
        saidaZ = tipping.output['VAngular']
        saidaX = tipping.output['VLinear']
        #print(saidaX)
        saidaAtivacao = tipping.output['Ativacao']
        saidaGarra = 180 - (valorG * 60)/100 # convertido para 180 ~ 120 da garra
        saidaUpdown = yik
        if valorU > 0.0:
            saidaUpdown = (valorU * 0.280)/100  
  
        #print(saidaAtivacao)   
        saidaBracoFrente = xik
        if saidaAtivacao > 75 and valorL > 0.0:
            saidaBracoFrente = (valorL * 0.280)/100
            #print(saidaBracoFrente)

        p = 0.03
        dx = abs(saidaBracoFrente - xik) 
        dy = abs(saidaUpdown - yik)
        
        pm = np.sqrt((dx*dx) + (dy*dy))#hipotenusa

        if pm > p:        
            xtik = (np.sign(saidaBracoFrente - xik) * p * np.cos(math.atan2(dy,dx))) + xik
            ytik =(np.sign(saidaUpdown - yik) * p* np.sin(math.atan2(dy,dx))) + yik
            if xtik > 0.280:
                xtik = 0.280
            if ytik > 0.280:
                ytik = 0.280
        

            angulos = IK(xtik,ytik)               
            move_group_arm(angulos) 

        msg.angular.z  = saidaZ
        msg.linear.x = saidaX
        msgTime = nowRosTime 

        cmd_vel.publish(msg)

if __name__ == '__main__':
    rospy.init_node('mediapipe_app', anonymous=True)
    init()
    rospy.spin()
    