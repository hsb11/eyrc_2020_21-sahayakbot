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

# This script is responsible to navigate the ebot to the desired position
class MoveBaseSeq():

    def __init__(self):

        rospy.init_node('move_base_task5')
        self.avoid = 0
        self.x_goal = 0
        self.y_goal = 0
        self.yaw_goal = 0
        # self.best_check = 0

        self.publisher = rospy.Publisher('/task5goal_received', Pose, queue_size=10)  # Acknowledgement topic to main script
        self.goal_receiver = Pose()
        self.goal_receiver.position.z = 0
        self.goal_receiver.position.x = 0
        self.publisher.publish(self.goal_receiver)

        self.goal_subsriber = rospy.Subscriber('/task5goal', Pose, self.goal_callback) # To get the navigation goal
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        #Create action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")

        self.rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            self.publisher.publish(self.goal_receiver)
            self.rate.sleep()

    def active_cb(self):
        rospy.loginfo("Goal pose is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        pass

    def done_cb(self, status, result):
        rospy.loginfo(status)

    # This function is responsible to check the status of ebot and provide acknowledgement to the main script when destination
    # has been reached
    def odom_callback(self, data):
        x_test = data.pose.pose.position.x
        y_test = data.pose.pose.position.y
        explicit_quat = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        y = euler_from_quaternion(explicit_quat)
        yaw_test = y[2]
        d = math.sqrt(((x_test-self.x_goal)**2)+((y_test-self.y_goal)**2))
        e_yaw = abs(yaw_test - self.yaw_goal)
        if d <= 0.3 and self.avoid == 1 and e_yaw <= 0.349066:
            self.goal_receiver.position.z = 0
            self.goal_receiver.position.x = 1  # Goal reached
            self.avoid = 0
            self.best_check = 0
            rospy.loginfo("Reached")

    # This function is responsible to initiate the navigation when asked to do so by the main script
    def goal_callback(self, dgoal):
        if dgoal.position.z == 1 and self.avoid == 0:
            self.goal_receiver.position.z = 1  # Goal acquired
            self.goal_receiver.position.x = 0
            self.x_goal = dgoal.position.x
            self.y_goal = dgoal.position.y
            explicit_quat1 = [dgoal.orientation.x, dgoal.orientation.y, dgoal.orientation.z, dgoal.orientation.w]
            ytemp = euler_from_quaternion(explicit_quat1)
            self.yaw_goal = ytemp[2]
            self.movebase_client(dgoal)
            self.avoid = 1

    def movebase_client(self, dgoal1):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        dgoal1.position.z = 0
        rospy.loginfo(dgoal1)
        goal.target_pose.pose = dgoal1
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)


if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
