#!/usr/bin/env python3  

import rospy
from tf2_msgs.msg import TFMessage
from apriltag_ros.msg import AprilTagDetectionArray
def getTopicAprilTag():
    global object_number  
    msg = rospy.wait_for_message("/tag_detections", AprilTagDetectionArray)
    for detection in msg.detections:       
        if detection.id[0] == 0:
        	return detection.pose.pose.pose.position.z
def callback(data):
    for transform in data.transforms:
        if transform.child_frame_id == "camera_struct":
           z = getTopicAprilTag()
           print(transform.transform.translation.z - z)
       
    
def listener():
    rospy.init_node('sensor_altra', anonymous=True)
    rospy.Subscriber("tf", TFMessage, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
