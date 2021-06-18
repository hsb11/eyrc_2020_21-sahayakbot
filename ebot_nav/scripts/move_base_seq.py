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

        
        rospy.init_node('move_base_seq')
        points_seq = rospy.get_param('move_base_seq/p_seq')
        
        # Only yaw angle required (no ratotions around x and y axes) in deg:
        yaweulerangles_seq = rospy.get_param('move_base_seq/yea_seq')
        #List of goal quaternions:
        quat_seq = list()
        #List of goal poses:
        self.pose_seq = list()
        self.goal_cnt = 0
        for yawangle in yaweulerangles_seq:
            #Unpacking the quaternion list and passing it as arguments to Quaternion message constructor
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        n = 3
        # Returns a list of lists [[point1], [point2],...[pointn]]
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        for point in points:
            #Exploit n variable to cycle in quat_seq
            self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
            n += 1
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
        self.movebase_client()
        rospy.spin()

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        #To print current pose at each feedback:
        #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")

    def done_cb(self, status, result):
        rospy.loginfo("WOW")
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
        p = [14.6, -0.8]
        d = math.sqrt(((x_test-p[0])**2)+((y_test-p[1])**2))
        e_yaw = abs(yaw_test - 0)
        if d <= 0.3 and e_yaw <= 0.0873:
            os.system("rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}")
            rospy.loginfo("Final goal pose reached!")
            rospy.signal_shutdown("Final goal pose reached!")
        # rospy.loginfo(x_test)
        # if self.goal_cnt == 0:
        #     p = [-9.1, -1.2]
        #     d = math.sqrt(((x_test-p[0])**2)+((y_test-p[1])**2))
        #     if d <= 0.5:
        #         self.goal_cnt = 1
        #         self.movebase_client()
        #     else:
        #         self.goal_cnt = 0
        # elif self.goal_cnt == 1:
        #     p = [10.7, 10.5]
        #     d = math.sqrt(((x_test-p[0])**2)+((y_test-p[1])**2))
        #     if d <= 0.5:
        #         self.goal_cnt = 2
        #         self.movebase_client()
        #     else:
        #         self.goal_cnt = 1
        # elif self.goal_cnt == 2:
        #     p = [12.6, -1.9]
        #     d = math.sqrt(((x_test-p[0])**2)+((y_test-p[1])**2))
        #     if d <= 0.5:
        #         self.goal_cnt = 3
        #         self.movebase_client()
        #     else:
        #         self.goal_cnt = 2
        # elif self.goal_cnt == 3:
        #     p = [18.2, -1.4]
        #     d = math.sqrt(((x_test-p[0])**2)+((y_test-p[1])**2))
        #     if d <= 0.5:
        #         self.goal_cnt = 4
        #         self.movebase_client()
        #     else:
        #         self.goal_cnt = 3
        # else:

        # else:
        #     self.goal_cnt = 4
        

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        #rospy.spin()


if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
