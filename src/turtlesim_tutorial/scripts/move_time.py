#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import sys

robot_x = 0
robot_y = 0

def pose_callback(pose):
    global robot_x, robot_y
    robot_x = pose.x
    robot_y = pose.y
    rospy.loginfo("Robot X = %f\t Robot Y = %f\n",pose.x, pose.y)

def move_turtle(lin_vel,ang_vel,distance):
    global robot_x, robot_y
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose',Pose, pose_callback)
    rate = rospy.Rate(1) # 10hz
    vel = Twist()
    while not rospy.is_shutdown():
        vel.linear.x = lin_vel
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = ang_vel
 
        if(robot_x >= distance or robot_y >= distance):
            rospy.loginfo("Robot hits a wall")
            rospy.logwarn("Robot Crashed")
            break
        pub.publish(vel)
        rate.sleep()

def move_rubot(lin_vel, ang_vel, distance, time_duration):
    vel = Twist()
    time_begin = rospy.Time.now()
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    blnMove = True
    #move_turtle(lin_vel, ang_vel, distance)
    while blnMove:
        duration = (rospy.Time.now() - time_begin).to_sec()
        if ( duration < time_duration):
            vel.linear.x = lin_vel
            vel.linear.y = 0
            vel.linear.z = 0
            vel.angular.x = 0
            vel.angular.y = 0
            vel.angular.z = ang_vel
            pub.publish(vel)
            rospy.loginfo("Robot Running")
        else:
            blnMove = False
            vel.linear.x = 0
            vel.linear.y = 0
            vel.linear.z = 0
            vel.angular.x = 0
            vel.angular.y = 0
            vel.angular.z = 0
            pub.publish(vel)
            #move_turtle(0,0,0)
            rospy.logwarn("Stopping Robot")
    rospy.loginfo("Robot ran for " + str(duration) + " seconds before stopping")            


if __name__ == '__main__':
    try:
        rospy.init_node('move_rubot', anonymous=False)#Has to be called here at the begining!
        v= rospy.get_param("~v")
        w= rospy.get_param("~w")
        d= rospy.get_param("~d")
        move_rubot(v,w,d,10)
    except rospy.ROSInterruptException:
        pass