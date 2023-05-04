import rospy
from visualization_msgs.msg import Marker,MarkerArray
import cv2
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from mediapipe_holistic_ros.msg import MediaPipeHolistic
from apriltag_ros.msg import AprilTagDetectionArray
import time
import random
rospy.init_node('rviz_hand_marker')
now = rospy.Time.now()
mao_marker_pub = rospy.Publisher("/mao", MarkerArray, queue_size = 2)
robot_marker_pub = rospy.Publisher("/robots", MarkerArray, queue_size = 2)
objects_marker_move_pub = rospy.Publisher("/objectmv", MarkerArray, queue_size = 2)
mao_marker = MarkerArray()
objects_marker = MarkerArray()
objects_marker_move = MarkerArray()
global maoEsquerda, maoDireita,faceL, pose, object_number
object_number = 0


def getTopicData():
    global maoEsquerda, maoDireita,faceL,pose
    maoEsquerda = []
    maoDireita = []
    faceL = []
    pose = []
    count = 0
    msg = rospy.wait_for_message("/MediaPipePose/holistic/landmarks", MediaPipeHolistic)
    #msg = msg.human_hand_list
    if (msg.left_hand_landmarks):
        for i in range(0,len(msg.left_hand_landmarks)):
            maoEsquerda.append(msg.left_hand_landmarks[i])

    if (msg.right_hand_landmarks):
        for i in range(0,len(msg.right_hand_landmarks)):
            maoDireita.append(msg.right_hand_landmarks[i])

    if (msg.face_landmarks):
        for i in range(0,len(msg.face_landmarks)):
            faceL.append(msg.face_landmarks[i])
    if (msg.pose_landmarks):
        for i in range(0,len(msg.pose_landmarks)):
            pose.append(msg.pose_landmarks[i])

def ajustePositionMarker(x,y):
	xb = x + 0.6
	xb = xb/1.2
	yb = y + 0.4
	yb = yb/0.8
	return xb,yb
	           
def getTopicAruco():
    global object_number  
    msg = rospy.wait_for_message("/tag_detections", AprilTagDetectionArray)
    for detection in msg.detections:
       
        if detection.id[0] == 0:
             print(detection.pose.pose.pose.position.x)
             x,y = ajustePositionMarker(detection.pose.pose.pose.position.x,detection.pose.pose.pose.position.y)
             objects_marker.markers[0].pose.position.x =x
             objects_marker.markers[0].pose.position.y =y
             objects_marker.markers[0].pose.position.z =0
             objects_marker.markers[0].pose.orientation.x = detection.pose.pose.pose.orientation.x
             objects_marker.markers[0].pose.orientation.y = detection.pose.pose.pose.orientation.y
             objects_marker.markers[0].pose.orientation.z = detection.pose.pose.pose.orientation.z
             objects_marker.markers[0].pose.orientation.w = detection.pose.pose.pose.orientation.w
             robot_marker_pub.publish(objects_marker)
             break
            # create_marker_object(0,transform)
      

# Set the scale of the marker
def create_markers(r):
	print(r)
	for i in range(0,r):
		 globals()['pt%s_marker' % i] = Marker()
		 globals()['pt%s_marker' % i].scale.x = 0.01
		 globals()['pt%s_marker' % i].scale.y = 0.01
		 globals()['pt%s_marker' % i].scale.z = 0.01

		 globals()['pt%s_marker' % i].color.r = 1.0
		 globals()['pt%s_marker' % i].color.g = 1.0
		 globals()['pt%s_marker' % i].color.b = 1.0
		 globals()['pt%s_marker' % i].color.a = 1.0

		 globals()['pt%s_marker' % i].pose.position.x = 0.0
		 globals()['pt%s_marker' % i].pose.position.y = 0.0
		 globals()['pt%s_marker' % i].pose.position.z = 0.0

		 globals()['pt%s_marker' % i].header.frame_id = "usb_cam"
		 globals()['pt%s_marker' % i].header.stamp = now
		 globals()['pt%s_marker' % i].type = 2
		 globals()['pt%s_marker' % i].id = i
		 mao_marker.markers.append( globals()['pt%s_marker' % i])
		 
		 
def create_marker_object(i):     
    #  x,y = ajustePositionMarker(transform.transform.translation.x,transform.transform.translation.y)
    #  print(x)
     globals()['robot%s' % i] = Marker()
     globals()['robot%s' % i].scale.x = 0.1
     globals()['robot%s' % i].scale.y = 0.1
     globals()['robot%s' % i].scale.z = 0.1

     globals()['robot%s' % i].color.r = 1.0
     globals()['robot%s' % i].color.g = 1.0
     globals()['robot%s' % i].color.b = 1.0
     globals()['robot%s' % i].color.a = 1.0

     globals()['robot%s' % i].pose.position.x = 0
     globals()['robot%s' % i].pose.position.y = 0
     globals()['robot%s' % i].pose.position.z = 0

            #marker do segundo robô
     globals()['robot%s' % i].header.frame_id = "usb_cam"
     globals()['robot%s' % i].header.stamp = now
     globals()['robot%s' % i].type = 1
     globals()['robot%s' % i].id = 600 + i
     objects_marker.markers.append( globals()['robot%s' % i])

def create_marker_object_move(i):     
	globals()['object%s_marker_move' % i] = Marker()
	globals()['object%s_marker_move' % i].scale.x = 0.1
	globals()['object%s_marker_move' % i].scale.y = 0.1
	globals()['object%s_marker_move' % i].scale.z = 0.1

	globals()['object%s_marker_move' % i].color.r = 1.0
	globals()['object%s_marker_move' % i].color.g = 0.0
	globals()['object%s_marker_move' % i].color.b = 1.0
	globals()['object%s_marker_move' % i].color.a = 1.0

	globals()['object%s_marker_move' % i].pose.position.x = objects_marker.markers[0].pose.position.x
	globals()['object%s_marker_move' % i].pose.position.y = objects_marker.markers[0].pose.position.y
	globals()['object%s_marker_move' % i].pose.position.z = objects_marker.markers[0].pose.position.z

	globals()['object%s_marker_move' % i].header.frame_id = "usb_cam"
	globals()['object%s_marker_move' % i].header.stamp = now
	globals()['object%s_marker_move' % i].type = 1
	globals()['object%s_marker_move' % i].id = 700+i
	objects_marker_move.markers.append( globals()['object%s_marker_move' % i])
	

     
create_markers(42)
create_marker_object(0)
getTopicAruco()
create_marker_object_move(0)


objects_marker_move_pub.publish(objects_marker_move)
def midpoint(x1, x2, y1, y2): 
    return ((x1 + x2)/2), ((y1 + y2)/2)

while(True):
    global maoEsquerda, maoDireita,faceL,pose,makercount
    markerlen = getTopicData()
    getTopicAruco()
    
    if maoEsquerda:
        for i in range(0,20):
          mao_marker.markers[i].pose.position.x = maoEsquerda[i].x
          mao_marker.markers[i].pose.position.y = maoEsquerda[i].y
          mao_marker.markers[i].pose.position.z = maoEsquerda[i].z

    else:
        for i in range(0,20):
          mao_marker.markers[i].pose.position.x = 0
          mao_marker.markers[i].pose.position.y = 0
          mao_marker.markers[i].pose.position.z = 0

    if maoDireita:
        for i in range(21,42):
          mao_marker.markers[i].pose.position.x = maoDireita[i-21].x
          mao_marker.markers[i].pose.position.y = maoDireita[i-21].y
          mao_marker.markers[i].pose.position.z = maoDireita[i-21].z
    
    else:
        for i in range(21,42):
          mao_marker.markers[i].pose.position.x = 0
          mao_marker.markers[i].pose.position.y = 0
          mao_marker.markers[i].pose.position.z = 0

    mao_marker_pub.publish(mao_marker)
    
    
    if maoEsquerda: 
        diff=np.sqrt((maoEsquerda[8].x - maoEsquerda[4].x) **2 + (maoEsquerda[8].y - maoEsquerda[4].y) **2 )
        mpx,mpy = midpoint(maoEsquerda[8].x,maoEsquerda[4].x,maoEsquerda[8].y,maoEsquerda[4].y)

        
        for i in range(1): 
            if diff <= objects_marker_move.markers[i].scale.x:           
                if (mpx < objects_marker_move.markers[i].pose.position.x + 0.05 and mpx > objects_marker_move.markers[i].pose.position.x - 0.05
                    and mpy < objects_marker_move.markers[i].pose.position.y + 0.05 and mpy > objects_marker_move.markers[i].pose.position.y - 0.05):
                    objects_marker_move.markers[i].pose.position.x = mpx
                    objects_marker_move.markers[i].pose.position.y = mpy
                    objects_marker_move.markers[i].color.r = 1
                    objects_marker_move.markers[i].color.g = 0  
                    objects_marker_move.markers[i].color.b = 0
                    objects_marker_move.markers[i].color.a = 1.0
            else:
                objects_marker_move.markers[i].color.r = 1
                objects_marker_move.markers[i].color.r = 1  
                objects_marker_move.markers[i].color.b = 1

            objects_marker_move_pub.publish(objects_marker_move)

    
   
   


