#!/usr/bin/python3

#################################################################################################
# Este codigo foi desenvolvido por Marco Teixeira: mantonio.t91@gmail.com 
#################################################################################################

import rospy
import cv2
import pcl
import ros_numpy
import mediapipe as mp
from sensor_msgs.msg import Image, PointCloud2,CameraInfo
import numpy as np
from cv_bridge import CvBridge
from mediapipe_holistic_ros.msg import  MediaPipeHolistic
from mediapipe_holistic_ros.msg  import  MediaPipePose
import message_filters
from sensor_msgs import point_cloud2
from PIL import Image as img
import freenect
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_holistic = mp.solutions.holistic
mp_hands = mp.solutions.hands

#####
input_usb_cam_topic = rospy.get_param("/mediapipe_holistic_ros/input_usb_cam_topic","/camera/rgb/image_color")
output_image_topic = rospy.get_param("/mediapipe_holistic_ros/output_image_topic","/mediapipe_holistic/output_image")
output_mediapipe_topic = rospy.get_param("/mediapipe_holistic_ros/output_mediapipe_topic","/MediaPipePose/holistic/landmarks")
pub_landmark_output = rospy.get_param("/mediapipe_holistic_ros/pub_landmark_output",True)
pub_image_output = rospy.get_param("/mediapipe_holistic_ros/pub_image_output",True)
view_image = rospy.get_param("/mediapipe_holistic_ros/open_view_image",True)
#####


def apply_landmark(image, results):     
        if (pub_image_output == True or view_image == True):
                # Draw landmark annotation on the image.
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                mp_drawing.draw_landmarks(
                        image,
                        results.face_landmarks,
                        mp_holistic.FACEMESH_CONTOURS,
                        landmark_drawing_spec=None,
                        connection_drawing_spec=mp_drawing_styles
                        .get_default_face_mesh_contours_style())
                mp_drawing.draw_landmarks(
                        image,
                        results.pose_landmarks,
                        mp_holistic.POSE_CONNECTIONS,
                        landmark_drawing_spec=mp_drawing_styles
                        .get_default_pose_landmarks_style())
                mp_drawing.draw_landmarks(
                        image,
                        results.right_hand_landmarks,
                        mp_hands.HAND_CONNECTIONS,
                        mp_drawing_styles.get_default_hand_landmarks_style(),
                        mp_drawing_styles.get_default_hand_connections_style())       
                mp_drawing.draw_landmarks(
                        image,
                        results.left_hand_landmarks,
                        mp_hands.HAND_CONNECTIONS,
                        mp_drawing_styles.get_default_hand_landmarks_style(),
                        mp_drawing_styles.get_default_hand_connections_style())
                        
        if (pub_image_output == True):
                image_message = bridge.cv2_to_imgmsg(image, encoding="passthrough")
                publisher_output_image.publish(image_message) 
        
        if (view_image == True):
                cv2.imshow('MediaPipe Holistic', cv2.flip(image, 1))
                if cv2.waitKey(5) & 0xFF == 27:
                        print("OK")                               
        return image    
        

       
def depth_to_xyz(u,v,depth_val,cam_intrin):
        """  xyz from u,v image coords """
        '''
        u - x image coordinate
        v - y image coodrinate
        depth_val - depth value at that (u,v) from depth_image
        '''

        fx=cam_intrin[0]
        fy=cam_intrin[4]
        cx=cam_intrin[2]
        cy=cam_intrin[5]

        z = float(depth_val)
        x = float((u - cx)/fx)*z
        y = float((v - cy)/fy)*z

        result = [x, y, z]
        print(result)
        return result

                             
def pub_results(results,detph,h,w):
  
        # #xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(detph)
        # xyz_array = point_cloud2.read_points(detph, 
        #                         field_names=("x", "y", "z"), 
        #                         skip_nans=True,
        #                         )
        
        
     
        print(h)
        print(w)        
        if (pub_landmark_output == True):
                landmarks = MediaPipeHolistic() 

                #face_landmarks
                if results.face_landmarks:
                        p_id = 0
                        for i in results.face_landmarks.landmark:
                                pose = MediaPipePose()
                                pose.id = p_id
                                pose.x = i.x
                                pose.y = i.y
                                pose.z = i.z
                                p_id += 1
                                landmarks.face_landmarks.append(pose)
                #left_hand_landmarks
                if results.left_hand_landmarks:
                        p_id = 0
                        for i in results.left_hand_landmarks.landmark:
                                depth_val = 0.0
                                if int(i.y * 480) <480 and int(i.x * 640) < 640:
                                        depth_val=float(detph[int(i.y * 480), int(i.x * 640)])
                                
                                #pts_x,pts_y,pts_z=depth_to_xyz(int(i.x * 640),int(i.y * 480),depth_val,cam_intrin)
                                # gen = point_cloud2.read_points_list(detph, field_names=("z"), skip_nans=True, uvs=[(int(i.x * 640), int(i.y * 480))])
                                # #teste = next(gen)
                                # for x in gen:
                                #     print(x[0])
                                #     z = x[0]
                                #     break
                                pose = MediaPipePose()
                                pose.id = p_id
                                pose.x = i.x 
                                pose.y = i.y
                                pose.z = depth_val 
                                p_id += 1
                                landmarks.left_hand_landmarks.append(pose)  
                                
                #right_hand_landmarks
                
                if results.right_hand_landmarks:
                        p_id = 0                        
                        for i in results.right_hand_landmarks.landmark:
                                
                                depth_val = 0.0
                                if int(i.y * 480) <480 and int(i.x * 640) < 640:
                                        depth_val=float(detph[int(i.y * 480), int(i.x * 640)])
                                # for xyz in xyz_array: 
                                            
                                #     if (int(xyz[0]*640)) == i.x*640  and (int(xyz[1]*480)) == i.y*480: 
                                #          print("W")           
                                #          z = xyz[2] 
                                #          break;
                                
                                pose = MediaPipePose()
                                pose.id = p_id
                                pose.x = i.x
                                pose.y = i.y
                                pose.z = depth_val
                                p_id += 1
                                landmarks.right_hand_landmarks.append(pose) 
                                
                                
                #pose_landmarks
                if results.pose_landmarks:
                        p_id = 0
                        for i in results.pose_landmarks.landmark:
                                pose = MediaPipePose()
                                pose.id = p_id
                                pose.x = i.x
                                pose.y = i.y
                                pose.z = i.z
                                pose.visibility = i.visibility
                                p_id += 1
                                landmarks.pose_landmarks.append(pose) 
                                
                #pose_world_landmarks
                if results.pose_world_landmarks:
                        p_id = 0
                        for i in results.pose_world_landmarks.landmark:
                                pose = MediaPipePose()
                                pose.id = p_id
                                pose.x = i.x
                                pose.y = i.y
                                pose.z = i.z
                                pose.visibility = i.visibility
                                p_id += 1
                                landmarks.pose_world_landmarks.append(pose)                 
                                                        
                publisher_output_mediapipe.publish(landmarks) 
        
def image_callback(rgb_data,depth_data,camera_data):
       
        with mp_holistic.Holistic(
            enable_segmentation=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as holistic:
                image = bridge.imgmsg_to_cv2(rgb_data, desired_encoding='bgra8')
                d_img= bridge.imgmsg_to_cv2(depth_data,"passthrough")
                
                image.flags.writeable = False
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                results = holistic.process(image)
                
                #apply landmark
                image = apply_landmark(image, results) 
                h,w,c=image.shape
                depth_arr=np.array(d_img,dtype=np.float32)
               
                cam_intrin=list(camera_data.K) 
                pub_results(results,depth_arr,cam_intrin,h,w)

	
if __name__ == '__main__':
        rospy.init_node('MediaPiPeHolistic')
        
        #Global Topics
        bridge = CvBridge()
        rospy.Rate(30)
        #Sub and Pub
        publisher_output_image = rospy.Publisher(output_image_topic, Image,queue_size = 30)
        publisher_output_mediapipe = rospy.Publisher(output_mediapipe_topic, MediaPipeHolistic, queue_size = 30)
        
        #rgb_image_sub = message_filters.Subscriber("/camera/rgb/image_rect_color", Image)
        #depth_image_sub = message_filters.Subscriber("/camera/depth/image_raw",Image)
        #camera_info_sub = message_filters.Subscriber("/camera/depth/camera_info",CameraInfo)
        
        #ts = message_filters.ApproximateTimeSynchronizer([rgb_image_sub, depth_image_sub,camera_info_sub], queue_size=10, slop=5, allow_headerless=True)
        #ts.registerCallback(image_callback)
        
        #rospy.spin()
        
        
        with mp_holistic.Holistic(
             enable_segmentation=False,
             min_detection_confidence=0.5,
             min_tracking_confidence=0.5) as holistic:
           while not rospy.is_shutdown():
                 try:
                         image = freenect.sync_get_video()[0]
                         image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
                         #data = image                       
                         #image = bridge.imgmsg_to_cv2(data, desired_encoding='bgra8')
                         depth_image_sub = freenect.sync_get_depth()[0]
                         
                 except:
                         print("Image read error")
                         continue                
                
                 #apply holistic
                 depth_arr=np.array(depth_image_sub,dtype=np.float32)
                 image.flags.writeable = False
                 image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                 results = holistic.process(image)
                 h,w,c=image.shape
                 #apply landmark
                 image = apply_landmark(image, results)  
                 pub_results(results,depth_arr,h,w)
                	 

                   
             

