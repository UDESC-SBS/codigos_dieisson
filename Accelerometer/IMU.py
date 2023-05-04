import rospy 
import socket
import tf 
from sensor_msgs.msg import Imu   
from geometry_msgs.msg import Vector3Stamped,Vector3
from nav_msgs.msg import Odometry
from math import sqrt,atan2,cos,sin,pi,atan

def imu_publisher(sock):
	br = tf.TransformBroadcaster()
	host="10.70.39.230"
	port=5555 
	theta = 0 
	gyro_x_offset = 0.0 
	gyro_y_offset = 0.0 
	gyro_z_offset = 0.0 
	acl_x_offset = 0.0 
	acl_y_offset = 0.0 
	acl_z_offset = 0.0 
	mag_x_offset = 0.0 
	mag_y_offset = 0.0 
	mag_z_offset = 0.0 
	pub_freq = 10 
	alpha = 0.9 
	count = 0 
	num_callibration_itrs = 60 
	debug = False
	
	mag_pub = rospy.Publisher('imu/mag', Vector3Stamped, queue_size=10)
	acl_pub = rospy.Publisher('imu/accel', Vector3Stamped, queue_size=10)
	gyro_pub = rospy.Publisher('imu/gyro', Vector3Stamped, queue_size=10)
	imu_pub = rospy.Publisher('imu', Imu, queue_size=10)
	rospy.init_node('imu_publisher', anonymous=True)
	rate = rospy.Rate(pub_freq)
	if rospy.has_param('~num_callibration_itrs'):
	    num_callibration_itrs = rospy.get_param('~num_callibration_itrs')
	if rospy.has_param('~host'):
	    host = rospy.get_param('~host')
	if rospy.has_param('~debug'):
	    debug = rospy.get_param('~debug')

	sock.bind((host,port))

	current_time = rospy.Time.now()
	last_time = rospy.Time.now()

	rospy.loginfo("waiting for device...")
	while not rospy.is_shutdown():
	    data,addr = sock.recvfrom(1024)
	    data = data.decode()
	    line = data.split(',')
	    print(len(line))
	    if len(line) == 9:  #received complete packet
	       current_time = rospy.Time.now()
	       gyro_x = float(line[6])
	       gyro_y = float(line[7])
	       gyro_z = float(line[8]) 
	       acl_x = float(line[0])
	       acl_y = float(line[1])
	       acl_z = float(line[2])
	       mag_x = float(line[3])
	       mag_y = float(line[4])
	       mag_z = float(line[5])
	       if count < num_callibration_itrs:
	          gyro_x_offset += gyro_x
	          gyro_y_offset += gyro_y
	          gyro_z_offset += gyro_z
	          acl_x_offset += acl_x
	          acl_y_offset += acl_y
	          acl_z_offset += acl_z
	          mag_x_offset += mag_x
	          mag_y_offset += mag_y
	          mag_z_offset += mag_z
	          count += 1
	       elif count == num_callibration_itrs and num_callibration_itrs != 0:
	          gyro_x_offset /= num_callibration_itrs
	          gyro_y_offset /= num_callibration_itrs
	          gyro_z_offset /= num_callibration_itrs
	          acl_x_offset /= num_callibration_itrs
	          acl_y_offset /= num_callibration_itrs
	          acl_z_offset /= num_callibration_itrs
	          mag_x_offset /= num_callibration_itrs
	          mag_y_offset /= num_callibration_itrs
	          mag_z_offset /= num_callibration_itrs
	          rospy.loginfo("finished callibrating yaw")
	          count += 1
	       else:
	          gyro_x -= gyro_x_offset
	          gyro_y -= gyro_y_offset
	          gyro_z -= gyro_z_offset
	          acl_x -= acl_x_offset
	          acl_y -= acl_y_offset
	          acl_z -= acl_z_offset
	          mag_x -= mag_x_offset
	          mag_y -= mag_y_offset
	          mag_z -= mag_z_offset
	          if debug:
	             rospy.loginfo('x %s y %s z %s', gyro_x, gyro_y, gyro_z)
	             
	          now = rospy.Time.now()
	          gyro_msg = Vector3Stamped()
	          gyro_msg.header.stamp = now
	          gyro_msg.header.frame_id = 'gyro'
	          gyro_msg.vector.x = gyro_x
	          gyro_msg.vector.y = gyro_y
	          gyro_msg.vector.z = gyro_z
	          gyro_pub.publish(gyro_msg)
	          
	          acl_msg = Vector3Stamped()
	          acl_msg.header.stamp = now
	          acl_msg.header.frame_id = 'accel'
	          acl_msg.vector.x = acl_x
	          acl_msg.vector.y = acl_y
	          acl_msg.vector.z = acl_z
	          acl_pub.publish(acl_msg)
	          
	          mag_msg = Vector3Stamped()
	          mag_msg.header.stamp = now
	          mag_msg.header.frame_id = 'mag'
	          mag_msg.vector.x = acl_x
	          mag_msg.vector.y = acl_y
	          mag_msg.vector.z = acl_z
	          mag_pub.publish(mag_msg)
	                
	          dt = rospy.Time.now().to_sec() - now.to_sec()
	          theta += dt*gyro_z
	          imu_msg = Imu()
	          imu_msg.header.stamp = now
	          imu_msg.header.frame_id = '/imu'
	          
	          q = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)	         
	          imu_msg.orientation.x = q[0]
	          imu_msg.orientation.y = q[1]
	          imu_msg.orientation.z = q[2]
	          imu_msg.orientation.w = q[3]
	          imu_msg.angular_velocity = gyro_msg.vector
	          imu_msg.linear_acceleration = acl_msg.vector
	          imu_msg.orientation_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
	          imu_msg.angular_velocity_covariance[0] = -1
	          imu_msg.linear_acceleration_covariance[0] = -1
	          imu_pub.publish(imu_msg)
	          last_time = current_time
	          #odom = rospy.wait_for_message('/odom',Odometry)
	          #print(odom)
	          g = 10.0;
	          r = atan(acl_msg.vector.y/acl_msg.vector.z);
	          p = atan(-acl_msg.vector.x/(sqrt(pow(acl_msg.vector.y,2)+pow(acl_msg.vector.z,2))));
	          br.sendTransform((0, 0, 0),tf.transformations.quaternion_from_euler(r, p, 0),now,"imu","imu_frame")
	          br.sendTransform((0, 0, 0),tf.transformations.quaternion_from_euler(r, p, 0),now,"gyro","imu")
	          br.sendTransform((0, 0, 0),tf.transformations.quaternion_from_euler(r, p, 0),now,"accel","imu")
	          br.sendTransform((0, 0, 0),tf.transformations.quaternion_from_euler(r, p, 0),now,"mag","imu")
                   
	    else:
	       rospy.loginfo("received incomplete UDP packet from android IMU")
	       continue
    
if __name__ == '__main__': 

   try: 
      sock=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
      sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
      imu_publisher(sock) 
   except rospy.ROSInterruptException: 
   	pass
        
