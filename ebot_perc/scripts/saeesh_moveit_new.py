#!/usr/bin/env python

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf import TransformListener
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import os
from std_msgs.msg import *

class Ur5Moveit():

    # Constructor
    def __init__(self):

        rospy.init_node('moveit_perc', anonymous=True)

        self.ur5_pose_1 = geometry_msgs.msg.Pose()
        self.t = TransformListener()

        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        self._gripper = "ur5_gripper"
        self._hand_group = moveit_commander.MoveGroupCommander(self._gripper)

        self._curr_state = self._robot.get_current_state()

        rospy.Subscriber('/arm_pose_goal', Pose,self.arm_pose_goal_callback)
        self.custom_pose_goal = Pose()
        self.got_pose_goal = 0

        rospy.Subscriber('/arm_named_goal', String,self.arm_named_goal_callback)
        self.named_goal = String()
        self.got_named_goal = 0

        rospy.Subscriber('/arm_order', Int8, self.arm_order_callback)
        self.given_order = 0 #0 to move to named pose ,1 to move to a custom pose, 2 for pick,3 for place, 4 for detect, else for nothing
        self.got_given_order = 0#Set 1 if got the order to prevent reassigning same order

        self.arm_status_pub = rospy.Publisher('/arm_status', Int8 ,queue_size = 10)#0 for idle 1 for busy 2 for done
        self.arm_current_status = 0

        self.current_obj_pub =  rospy.Publisher('/current_obj',String,queue_size = 10)#Send the object detected or picked, send none if placed or nothing detected
        self.current_arm_obj = 'none'# The object which the arm is carrying, 'none' for empty
        self.current_detect_obj = 'none'

        self.rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            # self.moveit_publisher.publish(self.moveit_goal_receiver)
            # self.moveit_publisher_special.publish(self.moveit_special_receiver)
            self.rate.sleep()

    def go_to_pose(self, arg_pose):
        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move
        self.got_pose_goal = 0
        return flag_plan

    def clear(self):
        self._group.clear_pose_targets()

    def go_to_named_pose(self, arg_pose):
        self._group.set_named_target(arg_pose)
        self._group.go(wait=True)
        self.got_named_goal = 0

    def detect(self):
        pass

    def pick(self):#, arg):
        self._hand_group.set_named_target(arg)
        self._hand_group.go(wait=True)

    def place(self):
        self._hand_group.set_named_target("open")
        self._hand_group.go(wait=True)

    def arm_pose_goal_callback(self,data):
        if(self.got_pose_goal == 0):
            self.custom_pose_goal = data
            self.got_pose_goal = 1

    def arm_named_goal_callback(self,data):
        if(self.got_named_goal == 0):
            self.named_goal = data.data
            rospy.loginfo(self.named_goal)
            self.got_named_goal = 1


    def arm_order_callback(self, data):
        self.given_order = data.data
        if(self.given_order >= 0 and self.given_order <=4 and self.got_given_order == 0):

            self.got_given_order = 1
            self.arm_status_pub.publish(1)
            if(self.given_order == 0 and self.got_given_order == 1):
                self.go_to_pose(self.custom_pose_goal)
            elif(self.given_order == 1 and self.got_given_order == 1):
                rospy.loginfo('Got order')
                rospy.loginfo(self.got_given_order)
                self.go_to_named_pose(self.named_goal)
            elif(self.given_order == 2 and self.got_given_order == 1):
                self.pick()
            elif(self.given_order == 3 and self.got_given_order == 1):
                self.place()
            elif(self.given_order == 4 and self.got_given_order == 1):
                self.coke()
                self.glue()
            self.arm_status_pub.publish(2)
            # self.arm_status_pub.publish(0)
            self.got_given_order = 0



        else:
            self.arm_status_pub.publish(0)




    def coke(self):
        rate1 = rospy.Rate(1)
        obj_ids = []#Give the object ids of coke in find_object_2d session
        check = 0
        for i in range(len(obj_ids)):
            if self.t.frameExists("ebot_base") and self.t.frameExists("object_"+str(obj_ids[i])) and check == 0:
                (self.coke, self.coke_rot) = self.t.lookupTransform("ebot_base", "object_"+str(obj_ids[i]), rospy.Time())
                check = 1

    def glue(self):
        rate1 = rospy.Rate(1)
        obj_ids = []#Give the object ids of glue in find_object_2d session
        check = 0
        for i in range(len(obj_ids)):
            if self.t.frameExists("ebot_base") and self.t.frameExists("object_"+str(obj_ids[i])) and check == 0:
                (self.glue, self.glue_rot) = self.t.lookupTransform("ebot_base", "object_"+str(obj_ids[i]), rospy.Time())
                check = 1

    def battery(self):
        rate1 = rospy.Rate(1)
        obj_ids = [10]#Give the object ids of glue in find_object_2d session
        check = 0
        for i in range(len(obj_ids)):
            if self.t.frameExists("ebot_base") and self.t.frameExists("object_"+str(obj_ids[i])) and check == 0:
                (self.battery, self.battery_rot) = self.t.lookupTransform("ebot_base", "object_"+str(obj_ids[i]), rospy.Time())
                check = 1

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    Ur5Moveit()
