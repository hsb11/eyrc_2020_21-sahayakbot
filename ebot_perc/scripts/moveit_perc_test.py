#! /usr/bin/env python

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf import TransformListener
import os
from object_msgs.msg import ObjectPose


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('moveit_perc_test', anonymous=True)

        # Initialization to publsh on the required topic
        # self.publisher = rospy.Publisher('/detection_info', ObjectPose, queue_size=10)
        # self.msg = ObjectPose()

        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        self._gripper = "ur5_gripper"
        self._hand_group = moveit_commander.MoveGroupCommander(self._gripper)

        self._curr_state = self._robot.get_current_state()

    def go_to_pose(self, arg_pose):
        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move
        return flag_plan

    # def pub(self, name, arg):
    #     self.msg.name = name
    #     self.msg.pose.pose.position.x = arg[0]
    #     self.msg.pose.pose.position.y = arg[1]
    #     self.msg.pose.pose.position.z = arg[2]
    #     self.publisher.publish(self.msg)

    def clear(self):
        self._group.clear_pose_targets()

    def go_to_named_pose(self, arg_pose):
        self._group.set_named_target(arg_pose)
        self._group.go(wait=True)

    def pick(self, arg):
        self._hand_group.set_named_target(arg)
        self._hand_group.go(wait=True)

    def place(self):
        self._hand_group.set_named_target("open")
        self._hand_group.go(wait=True)

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()


def main():
    ur5 = Ur5Moveit()
    ur5_pose_1 = geometry_msgs.msg.Pose()

    # Home position
    # ur5.go_to_named_pose("home_pantry_straight")
    # ur5.place()

    q1 = quaternion_from_euler(-2.701, 0, 0)
    ur5_pose_1.orientation.x = q1[0]
    ur5_pose_1.orientation.y = q1[1]
    ur5_pose_1.orientation.z = q1[2]
    ur5_pose_1.orientation.w = q1[3]

    ####  FIRST
    ur5_pose_1.position.x = 0.3
    ur5_pose_1.position.y = -0.3471 + 0.18 + 0.1
    ur5_pose_1.position.z = 0.95 + 0.2
    flag = ur5.go_to_pose(ur5_pose_1)
    flag = ur5.go_to_pose(ur5_pose_1)
    flag = ur5.go_to_pose(ur5_pose_1)

    ######  Grabbing position
    # ur5_pose_1.position.x = 0.732 - 0.2
    # ur5_pose_1.position.y = -0.3471 - 0.015
    # ur5_pose_1.position.z = 0.8093 + 0.1
    
    # flag = ur5.go_to_pose(ur5_pose_1)
    # by = 0.05
    # while not flag:
    #     ur5_pose_1.position.x =  - 0.2 - by
    #     # self.ur5_pose_1.position.y = self.coke[1] - 0.005 - by
    #     flag = ur5.go_to_pose(ur5_pose_1)
    #     by = by + 0.05


    # CLOSE & PICK
    # ur5.pick("close_coke")

    # # MOVE AWAY
    # ur5_pose_1.position.x = coke[0] - 0.2 - 0.15
    # ur5_pose_1.position.y = coke[1] - 0.005  # -0.05 to compensate y dist.
    # ur5_pose_1.position.z = coke[2] + 0.2
    # flag = ur5.go_to_pose(ur5_pose_1)

    # # NAV HOME POSITION
    # ur5.go_to_named_pose("home")



    rate = rospy.Rate(1)
    # q1 = quaternion_from_euler(-2.61799, 0, math.pi)  # Tool orientation for the task
    while not rospy.is_shutdown():
        # rospy.spin()
        rate.sleep()


if __name__ == '__main__':
    main()
