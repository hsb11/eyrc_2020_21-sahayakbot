#!/usr/bin/env python

import rospy
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionFeedback
import os
from std_msgs.msg import *


class MoveBaseSeq():
    def __init__(self):
        #Create action client

        rospy.init_node('move_base_seq')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")

        self.goal_received = Pose()
        self.status = 0 # 0 for idle 1 for busy
        self.has_goal = 0 # To prevent getting repetitive goals

        rospy.Subscriber('/base_goal', Pose, self.goal_callback)
        self.status_pub = rospy.Publisher('/move_base_status', Int8 ,queue_size = 10)
        # self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)



        self.rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            self.rate.sleep()
        # self.movebase_client(self.x_goal, self.y_goal)
        rospy.spin()

    def active_cb(self):
        self.status_pub.publish(1)
        self.status = 1
        rospy.loginfo("Goal pose is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        #To print current pose at each feedback:
        #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
        rospy.loginfo("Feedback for goal pose received")

    def done_cb(self, status, result):
        rospy.loginfo("WOW")
        self.status = 0
        #rospy.loginfo(status)
        self.status_pub.publish(2)
        self.status_pub.publish(0)
        self.has_goal = 0

    def goal_callback(self, data):
        #rospy.loginfo('Got Goal')
        if(self.has_goal == 0):
            self.has_goal = 1
            self.movebase_client(data)

    def movebase_client(self, data):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        rospy.loginfo(data)
        goal.target_pose.pose = data
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)

if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
