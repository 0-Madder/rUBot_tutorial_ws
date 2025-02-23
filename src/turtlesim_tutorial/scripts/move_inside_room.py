#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import sys

robot_x = 5.5
robot_y = 5.5

def pose_callback(pose):
    global robot_x, robot_y
    robot_x = pose.x
    robot_y = pose.y
    rospy.loginfo("Robot X = %f\t Robot Y = %f\n",pose.x, pose.y)

def move_turtle(lin_vel,ang_vel,distance_x_r, distance_x_l, distance_y_u, distance_y_d):
    global robot_x, robot_y
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose',Pose, pose_callback)
    rate = rospy.Rate(10) # 10hz
    vel = Twist()
    while not rospy.is_shutdown():
        vel.linear.x = lin_vel
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = ang_vel
 
        if(robot_x >= distance_x_r or robot_y >= distance_y_u or robot_x < distance_x_l or robot_y < distance_y_d):
            rospy.loginfo("Robot hits a wall")
            rospy.logwarn("Stopping robot")
            break
        pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('move_turtle', anonymous=False)#Has to be called here at the begining!
        v= rospy.get_param("~v")
        w= rospy.get_param("~w")
        dx_r= rospy.get_param("~dx_r")
        dx_l= rospy.get_param("~dx_l")
        dy_u= rospy.get_param("~dy_u")
        dy_d= rospy.get_param("~dy_d")
        move_turtle(v,w,dx_r, dx_l, dy_u, dy_d)
    except rospy.ROSInterruptException:
        pass