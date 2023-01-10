#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

from math import atan2,sqrt,pow
import numpy as np
import time
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry



def odomCallback(pos_msg):
    global x_now
    global y_now
    global yaw
    x_now= pos_msg.pose.pose.position.x
    y_now= pos_msg.pose.pose.position.y
    rot_coord = pos_msg.pose.pose.orientation
    #total bakwass msg
    (roll,pitch,yaw)=euler_from_quaternion([rot_coord.x, rot_coord.y, rot_coord.z, rot_coord.w])
   


def goToGoal(velocity_pub, x_goal, y_goal):
    global x_now
    global y_now, yaw
    vel_message = Twist()

    while (True):
        Kp_linear = 0.1
        distance = abs(sqrt(((x_goal-x_now) ** 2) + ((y_goal-y_now) ** 2)))

        linear_speed = distance * Kp_linear

        Kp_angular = 1.0
        desired_angle_goal = atan2(y_goal-y_now, x_goal-x_now)
        angular_speed = (desired_angle_goal-yaw)*Kp_angular

        vel_message.linear.x = linear_speed
        vel_message.angular.z = angular_speed

        velocity_pub.publish(vel_message)
        print ('x=', x_now, ', y=',y_now, ', distance to goal: ', distance)

        if (distance <0.1):
            break


if __name__ == '__main__':
    
    rospy.init_node('turtlebot_follow_pts')
    

        #declare velocity publisher
        
    velocity_pub= rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
    
    pos_sub = rospy.Subscriber('/odom', Odometry, odomCallback) 
    time.sleep(2)
    rate=rospy.Rate(0.5)
    goToGoal(velocity_pub,0,0)
    time.sleep(1)
    goToGoal(velocity_pub,15,0)
    time.sleep(2)
    goToGoal(velocity_pub,0,-20)
    time.sleep(2)
    goToGoal(velocity_pub,15,-20)
    time.sleep(2)
    
    
    


