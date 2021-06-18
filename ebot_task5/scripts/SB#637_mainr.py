#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import math
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import threading
from geometry_msgs.msg import Twist

# This script is used to meet additional rulebook requirements

pose = [0, 0]  # robot odom pose at any time
rooms = [0, 0, 0, 0, 0, 0]  # rooms flag


def odom_callback(data):
    global pose
    pose = [data.pose.pose.position.x, data.pose.pose.position.y]


def rooms_display():
	global rooms
	if not rooms[0]:
		d = math.sqrt(((8.628-pose[0])**2)+((2.5-pose[1])**2))
		if d <= 0.5:
			rospy.loginfo("Meeting Room Reached")
			rooms[0] = 1
	elif not rooms[1]:
		d = math.sqrt(((10.778-pose[0])**2)+((9.313-pose[1])**2))
		if d <= 0.5:
			rospy.loginfo("Research Lab Reached")
			rooms[1] = 1
	elif not rooms[2]:
		d = math.sqrt(((25.247-pose[0])**2)+((-2.721-pose[1])**2))
		if d <= 0.5:
			rospy.loginfo("Store Room Reached")
			rooms[2] = 1
	elif not rooms[3]:
		d = math.sqrt(((13-pose[0])**2)+((-0.7-pose[1])**2))
		if d <= 0.5:
			rospy.loginfo("Pantry Room Reached")
			rooms[3] = 1
	elif not rooms[4]:
		d = math.sqrt(((5.222-pose[0])**2)+((-0.5-pose[1])**2))
		if d <= 0.5:
			rospy.loginfo("Conference Room Reached")
			rooms[4] = 1
	elif rooms[4] == 1:
		d = math.sqrt(((0-pose[0])**2)+((0-pose[1])**2))
		if rooms[5] == 0 and d <= 0.3:
			rospy.loginfo("Mission Accomplished!")
			rooms[5] = 1


def main():

	rospy.init_node('mainr', anonymous=True)
	rospy.Subscriber('/odom', Odometry, odom_callback)

	rospy.loginfo("Started Run!")

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		rooms_display()
		rate.sleep()  # Run while loop at given rate


# Python Main
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
