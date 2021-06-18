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


class MoveBaseSeq():

    def __init__(self):

        rospy.init_node('move_base_task5')
        self.avoid = 0
        self.x_goal = 0
        self.y_goal = 0
        self.yaw_goal = 0
        # self.best_check = 0

        self.publisher = rospy.Publisher('/task5goal_received', Pose, queue_size=10)
        self.goal_receiver = Pose()
        self.goal_receiver.position.z = 0
        self.goal_receiver.position.x = 0
        self.publisher.publish(self.goal_receiver)

        self.goal_subsriber = rospy.Subscriber('/task5goal', Pose, self.goal_callback)
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
        # self.movebase_client(self.x_goal, self.y_goal)
        # rospy.spin()

    def active_cb(self):
        rospy.loginfo("Goal pose is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        #To print current pose at each feedback:
        #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
        rospy.loginfo("Feedback for goal pose received")

    def done_cb(self, status, result):
        rospy.loginfo("WOW")
        rospy.loginfo(status)
        # if status == 3:
            # self.best_check = 1

        #self.goal_cnt += 1
    # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        # if status == 2:
        #     rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        # if status == 3:
        #     rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached") 
        #     if self.goal_cnt< len(self.pose_seq):
        #         next_goal = MoveBaseGoal()
        #         next_goal.target_pose.header.frame_id = "map"
        #         next_goal.target_pose.header.stamp = rospy.Time.now()
        #         next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        #         rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        #         rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        #         self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb) 
        #     else:
        #         rospy.loginfo("Final goal pose reached!")
        #         rospy.signal_shutdown("Final goal pose reached!")
        #         return

        # if status == 4:
        #     rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
        #     rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
        #     return

        # if status == 5:
        #     rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
        #     rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
        #     return

        # if status == 8:
        #     rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def odom_callback(self, data):
        x_test = data.pose.pose.position.x
        y_test = data.pose.pose.position.y
        explicit_quat = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        y = euler_from_quaternion(explicit_quat)
        yaw_test = y[2]
        d = math.sqrt(((x_test-self.x_goal)**2)+((y_test-self.y_goal)**2))
        e_yaw = abs(yaw_test - self.yaw_goal)
        if d <= 0.3 and self.avoid == 1 and e_yaw <= 0.349066:
            # os.system("rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}")
        # if self.best_check == 1:
            self.goal_receiver.position.z = 0
            self.goal_receiver.position.x = 1  # Goal reached
            # self.publisher.publish(self.goal_receiver)
            self.avoid = 0
            self.best_check = 0
            rospy.loginfo("Reached")


    def goal_callback(self, dgoal):
        if dgoal.position.z == 1 and self.avoid == 0:
            self.goal_receiver.position.z = 1  # Goal acquired
            self.goal_receiver.position.x = 0
            # self.publisher.publish(self.goal_receiver)
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
        # goal.target_pose.pose.position.x = x
        # goal.target_pose.pose.position.y = y
        # rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        # rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        #rospy.spin()


if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
