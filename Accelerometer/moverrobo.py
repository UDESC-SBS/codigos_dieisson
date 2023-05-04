import math
import os
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


feedback = Pose()


def subCallback(msg):
    feedback.x = msg.x
    feedback.y = msg.y
    feedback.theta = msg.theta
    

def main():
    rospy.init_node('controle_pid', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/pose', Pose, callback=subCallback)
    
    rate = rospy.Rate(10)

    os.system('rosservice call /reset')

    posdesejada = []
    tolerancePos = 0.05
    toleranceOrient = 0.005

    kpos = 1.5
    ki = 0.000005
    kd = 0.000005

    korient = 4
    erro = 99
    erroInt = 99
    erroOrient = 99
    lastErro = 99
    
    posdesejada.append(float(input("Digite a posição X: ")))
    rospy.loginfo("Posição: %s" % posdesejada[0])
    
    posdesejada.append(float(input("Digite a posição Y: ")))
    rospy.loginfo("Posição: %s" % posdesejada[1])

    msg = Twist()
    last_time = None

    # Força a obtenção da pose atual
    pose_atual = rospy.wait_for_message('/turtle1/pose', Pose)
    subCallback(pose_atual)
    
    while abs(erroOrient) > toleranceOrient or abs(erro) > tolerancePos:
        if last_time is None:
            last_time = rospy.get_time()
        # variação no tempo
        actual_time = rospy.get_time() # in seconds
        dt = actual_time - last_time

        lastErro  = erro 

        # erro = disancia entre dois pontos
        erro = math.sqrt(math.pow(posdesejada[0] - feedback.x, 2) + math.pow(posdesejada[1]-feedback.y,2))
        erroInt = erroInt + (erro * dt)
        erroDer = (erro - lastErro) / dt

        # calcula a saída PID
        msg.linear.x = kpos * erro + ki * erroInt + kd * erroDer

        erroOrient = math.atan2(posdesejada[1] - feedback.y, posdesejada[0] - feedback.x) - feedback.theta
        msg.angular.z = korient * erroOrient
        rospy.loginfo('Theta>>%f,Erro>>%f' % (feedback.theta, erroOrient))
        rospy.loginfo('X>>%f,Erro>>%f' % (feedback.x, erro))

        pub.publish(msg)
        rate.sleep()

    msg.angular.z = 0
    msg.angular.x = 0
    pub.publish(msg)
    rospy.loginfo('Orientação alcançada...')
    rospy.loginfo('Posição alcançada...')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass