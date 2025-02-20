#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from turtlesim.msg import Pose
import sys

robot_x = 0
robot_y = 0

def pose_callback(pose):
    global robot_x, robot_y
    robot_x = pose.x
    robot_y = pose.y
    rospy.loginfo("Robot X = %f\t Robot Y = %f\n",pose.x, pose.y)


def move_rubot(lin_vel, ang_vel, distance, time_duration):
    vel = Twist() #Mi velocidad es un mensaje con la forma de un Twist
    vel.linear = Vector3(lin_vel, 0, 0)
    vel.angular = Vector3(0, 0, ang_vel) #Le doy los valores que he cogido previamente del .launch
    time_begin = rospy.Time.now() #Empiezo a contar el tiempo
    blnMove = True #boolean para controlar si el robot se mueve o no
    rospy.Subscriber('/turtle1/pose',Pose, pose_callback)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    while blnMove: #Cuando blnMove sea false, significa que el el robot ha parado, y el bucle se termina

        duration = (rospy.Time.now() - time_begin).to_sec()

        if(robot_x >= distance or robot_y >= distance): #Si me he chocado, paro el robot
            blnMove = False
            vel.linear = Vector3(0,0,0)
            vel.angular = Vector3(0,0,0)
            rospy.loginfo("Robot hits a wall")
            rospy.logwarn("Robot Crashed after running for " + str(duration) + "  seconds, what a shame")
            return

        elif ( duration < time_duration): #Si no he chocado y el tiempo aun no ha pasado, sigo moviendo el robot
            rospy.loginfo("Robot Running")
            
        else: #Si el tiempo ha pasado, paro el robot
            blnMove = False
            vel.linear = Vector3(0,0,0)
            vel.angular = Vector3(0,0,0)
            rospy.logwarn("Stopping Robot")
            rospy.loginfo("Robot ran for " + str(duration) + " seconds before stopping")  
            return    

        pub.publish(vel) #A cada iteracion del bucle actualizo la velocidad, por si ha cambiado en algun if


if __name__ == '__main__':
    try:
        rospy.init_node('move_rubot', anonymous=False)#Has to be called here at the begining!
        v= rospy.get_param("~v")
        w= rospy.get_param("~w")
        d= rospy.get_param("~d")
        t= rospy.get_param("~t")
        move_rubot(v,w,d,t)
    except rospy.ROSInterruptException:
        pass