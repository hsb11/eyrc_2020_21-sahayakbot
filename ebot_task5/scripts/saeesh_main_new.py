#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import math
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import threading
from geometry_msgs.msg import Twist
from std_msgs.msg import *

global goal_pub
base_goal_coord = Pose()
base_status = 0


global arm_pose_goal_pub,arm_named_goal_pub,arm_order_pub
current_arm_status = 0
current_arm_object = 'none'

def give_base_goal():
    global base_goal_coord

def base_status_callback(data):
    global base_status
    base_status = data.data

def arm_status_callback(data):
    global arm_status, current_arm_status
    current_arm_status = data.data

def current_obj_callback(data):
    global current_arm_obj
    current_arm_obj = data.data


def main():
    global goal_pub,base_goal_coord
    global base_status

    global arm_pose_goal_pub,arm_named_goal_pub,arm_order_pub
    global current_arm_status,current_arm_obj

    rospy.init_node('main', anonymous=True)

    goal_pub = rospy.Publisher('/base_goal', Pose,queue_size = 10)#Giving a custom Pose Goal for move_base
    rospy.Subscriber('/move_base_status', Int8 ,base_status_callback)

    arm_pose_goal_pub = rospy.Publisher('/arm_pose_goal', Pose,queue_size = 10)#Giving a custom Pose Goal for arm
    arm_named_goal_pub = rospy.Publisher('/arm_named_goal', String,queue_size = 10)
    arm_order_pub = rospy.Publisher('/arm_order', Int8,queue_size = 10)#0 to move to named pose ,1 to move to a custom pose, 2 for pick,3 for place, 4 for detect, else for nothing
    rospy.Subscriber('/arm_status', Int8 ,arm_status_callback)#Status fo arm
    rospy.Subscriber('/current_obj',String,current_obj_callback)#If the arm has picked ,placed or detected any object

    rospy.loginfo('Gave arm order')

    while current_arm_status != 2:
        arm_named_goal_pub.publish('allZeros2')
        rospy.sleep(1)
        arm_order_pub.publish(1)

    rospy.loginfo('Arm performed the task')

    # base_goal_coord.position.x = 8.628
    # base_goal_coord.position.y = 1
    # base_goal_coord.position.z = 0
    # o = quaternion_from_euler(0, 0, 1.57)
    # base_goal_coord.orientation.x = o[0]
    # base_goal_coord.orientation.y = o[1]
    # base_goal_coord.orientation.z = o[2]
    # base_goal_coord.orientation.w = o[3]
    # rospy.loginfo('Sent Goal Meeting')
    # goal_pub.publish(base_goal_coord)
    #
    # while base_status != 2:
    #     goal_pub.publish(base_goal_coord)
    # rospy.loginfo('Reached Goal Meeting')
    # rospy.sleep(2)
    #
    # base_goal_coord.position.x = 10.778
    # base_goal_coord.position.y = 9.313
    # base_goal_coord.position.z = 0
    # o = quaternion_from_euler(0, 0, 0)
    # base_goal_coord.orientation.x = o[0]
    # base_goal_coord.orientation.y = o[1]
    # base_goal_coord.orientation.z = o[2]
    # base_goal_coord.orientation.w = o[3]
    # rospy.loginfo('Sent Goal Research')
    # goal_pub.publish(base_goal_coord)
    #
    # while base_status != 2:
    #     goal_pub.publish(base_goal_coord)
    # rospy.loginfo('Reached Goal Research')

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()
        rospy.spin()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
