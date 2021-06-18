#! /usr/bin/env python

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf import TransformListener, TransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import os

# This script does the work of perception using find_object_2d and controlling the arm to pick and place.

pwp = 0
xyz = 0


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('moveit_perc', anonymous=True)
        self.one_time_home = 1
        self.avoid = 0
        self.avoid2 = 0
        self.avoid_dropped = 0
        self.proceed_with_picking = 0
        self.drop = 0
        self.drop_room = 'Empty'
        self.store_extra_home = 0
        self.ur5_pose_1 = geometry_msgs.msg.Pose()
        self.t = TransformListener ()
        self.br = TransformBroadcaster()

        self.moveit_publisher = rospy.Publisher('/moveit_goal_received', Pose, queue_size=10)
        self.moveit_goal_receiver = Pose()
        self.moveit_goal_receiver.position.z = 0  # Started moveit perc node flag
        self.moveit_goal_receiver.position.x = 0  # Picked up the object flag
        self.moveit_goal_receiver.position.y = 0  # Table side flag
        self.moveit_publisher.publish(self.moveit_goal_receiver)

        self.moveit_publisher_special = rospy.Publisher('special_return', Pose, queue_size=10)
        self.moveit_special_receiver = Pose()
        self.moveit_special_receiver.position.x = 0  # Picked up the object
        self.moveit_special_receiver.position.z = 0  # Dropped the object flag
        self.moveit_special_receiver.position.y = 0  # Image Window

        self.goal_subsriber = rospy.Subscriber('/moveit_goal', Odometry, self.moveit_goal_callback)
        self.special_subscriber = rospy.Subscriber('/special', Odometry, self.special_callback)

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

        self.store_loc_map = [0, 0, 0]
        self.rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            self.moveit_publisher.publish(self.moveit_goal_receiver)
            self.moveit_publisher_special.publish(self.moveit_special_receiver)
            self.br.sendTransform(self.store_loc_map,
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "store_object_frame",
                         "map")                  # Publishing a required tf frame
            self.rate.sleep()

    def special_callback(self, data):
        self.proceed_with_picking = data.pose.pose.position.y
        self.drop = data.pose.pose.position.z
        self.drop_room = data.header.frame_id
        if self.drop == 1 and self.avoid_dropped == 0:
            self.avoid_dropped = 1
            self.drop_the_object(self.drop_room)

    def drop_the_object(self, room):
        if room == 'Meeting':
            self.go_to_named_pose("dropbox_meeting_top_3")
            self.go_to_named_pose("dropbox_meeting_top_3")
            self.go_to_named_pose("dropbox_meeting_2")
            self.go_to_named_pose("dropbox_meeting_2")
            self.place()
            self.place()
            self.go_to_named_pose("dropbox_meeting_top_3")
            self.go_to_named_pose("dropbox_meeting_top_3")
            self.go_to_named_pose("home")
            self.go_to_named_pose("home")
            self.moveit_special_receiver.position.z = 1  # Dropped indication to main
            rospy.sleep(4.)
            self.moveit_special_receiver.position.z = 0

        elif room == 'Research':
            self.go_to_named_pose("dropbox_meeting_top_3")
            self.go_to_named_pose("dropbox_meeting_top_3")
            self.go_to_named_pose("dropbox_meeting_2")
            self.go_to_named_pose("dropbox_meeting_2")
            self.place()
            self.place()
            self.go_to_named_pose("dropbox_meeting_top_3")
            self.go_to_named_pose("dropbox_meeting_top_3")
            self.go_to_named_pose("home")
            self.go_to_named_pose("home")
            self.moveit_special_receiver.position.z = 1  # Dropped indication to main
            rospy.sleep(4.)
            self.moveit_special_receiver.position.z = 0

        elif room == 'Conference':
            self.go_to_named_pose("dropbox_meeting_top_3")
            self.go_to_named_pose("dropbox_meeting_top_3")
            self.go_to_named_pose("dropbox_meeting_2_conference")
            self.go_to_named_pose("dropbox_meeting_2_conference")
            self.place()
            self.place()
            self.go_to_named_pose("dropbox_meeting_top_3")
            self.go_to_named_pose("dropbox_meeting_top_3")
            self.go_to_named_pose("home")
            self.go_to_named_pose("home")
            self.moveit_special_receiver.position.z = 1  # Dropped indication to main
            rospy.sleep(4.)
            self.moveit_special_receiver.position.z = 0

    def moveit_goal_callback(self, data):
        self.store_extra_home = data.pose.pose.position.y
        if self.one_time_home == 1:
            self.go_to_named_pose("home")
            self.one_time_home = 0
        if data.pose.pose.position.z == 1 and self.avoid == 0:
            self.moveit_goal_receiver.position.z = 1  # moveit_perc started for that object in the room
            self.moveit_goal_receiver.position.x = 0
            self.moveit_special_receiver.position.x = 0
            self.avoid = 1
            self.moveit_start(data.header.frame_id)
        if data.header.frame_id == 'Coke' and self.avoid2 == 0 and data.pose.pose.position.z == 1:
            self.moveit_goal_receiver.position.z = 1  # moveit_perc started for that object in the room
            self.moveit_goal_receiver.position.x = 0
            self.moveit_special_receiver.position.x = 0
            self.avoid2 = 1
            self.moveit_start(data.header.frame_id)

    def moveit_start(self, object_2b_picked):
        global pwp, xyz
        object_detected_check = 0
        rospy.loginfo(object_2b_picked)

        # Store Room
        if object_2b_picked == 'FPGA Board' or object_2b_picked == 'eYFI Board' or object_2b_picked == 'Pair of Wheels Package' or object_2b_picked == 'Battery':
            self.go_to_named_pose("store_detection_right")
            rospy.sleep(0.5)
            self.moveit_special_receiver.position.y = 1  # Image Window Turn On
            object_detected_check = self.store_detect(object_2b_picked)
            side = 'R'

            rospy.sleep(7)
            self.moveit_special_receiver.position.y = 0  # Image Window Turn Off
            rospy.sleep(1)

            extra = 0
            if (object_detected_check == 0) or self.store_loc_map[1] > -3.35:
                self.go_to_named_pose("store_detection_left")
                extra = 1
                rospy.sleep(0.5)
                self.moveit_special_receiver.position.y = 1  # Image Window
                object_detected_check = self.store_detect(object_2b_picked)
                side = 'L'

                rospy.sleep(7)
                self.moveit_special_receiver.position.y = 0  # Image Window

            if extra == 0:
                self.go_to_named_pose("store_detection_left")
                rospy.sleep(0.5)
                self.moveit_special_receiver.position.y = 1  # Image Window
                rospy.sleep(7)
                self.moveit_special_receiver.position.y = 0  # Image Window

            rospy.loginfo(self.store_loc_map)
            self.go_to_named_pose("store_pickup_home_3_new")
            self.go_to_named_pose("store_pickup_home_3_new")

            if side == 'L':
                self.moveit_goal_receiver.position.y = 1  # 1 for left side
            else:
                self.moveit_goal_receiver.position.y = 2  # 2 for right side
            rospy.loginfo("STORE TABLE SIDE GIVEN TO MAIN")

            while pwp == 0:
                pwp = self.proceed_with_picking
                pass

            self.moveit_goal_receiver.position.y = 0
            rospy.sleep(3.)
            (self.store_loc_ebot_base, rotation) = self.t.lookupTransform("ebot_base", "store_object_frame", rospy.Time(0))
            rospy.loginfo(self.store_loc_ebot_base)

            self.go_to_named_pose("store_pickup_home_2")
            self.go_to_named_pose("store_pickup_home_2")

            q1 = quaternion_from_euler(-2.701, 0, 0)
            self.ur5_pose_1.orientation.x = q1[0]
            self.ur5_pose_1.orientation.y = q1[1]
            self.ur5_pose_1.orientation.z = q1[2]
            self.ur5_pose_1.orientation.w = q1[3]

            # FIRST POSITION
            rospy.loginfo("FIRST POSITION")
            self.ur5_pose_1.position.x = self.store_loc_ebot_base[0]
            self.ur5_pose_1.position.y = self.store_loc_ebot_base[1] + 0.3
            self.ur5_pose_1.position.z = self.store_loc_ebot_base[2] + 0.2
            flag = self.go_to_pose(self.ur5_pose_1)
            flag = self.go_to_pose(self.ur5_pose_1)
            if not flag:
                flag = self.go_to_pose(self.ur5_pose_1)

            # GRABBING POSITION
            rospy.loginfo("GRABBING POSITION")
            # self.ur5_pose_1.position.x = self.coke[0] - 0.18
            self.ur5_pose_1.position.y = self.store_loc_ebot_base[1] + 0.21
            self.ur5_pose_1.position.z = self.store_loc_ebot_base[2] + 0.11
            flag = self.go_to_pose(self.ur5_pose_1)
            flag = self.go_to_pose(self.ur5_pose_1)
            if not flag:
                flag = self.go_to_pose(self.ur5_pose_1)

            # CLOSE & PICK
            rospy.loginfo("PICK")
            self.pick("close4")

            # MOVE AWAY
            rospy.loginfo("MOVE AWAY")
            # self.ur5_pose_1.position.x = self.coke[0] - 0.18 - 0.15
            self.ur5_pose_1.position.y = self.store_loc_ebot_base[1] + 0.3 
            self.ur5_pose_1.position.z = self.store_loc_ebot_base[2] + 0.2
            flag = self.go_to_pose(self.ur5_pose_1)
            flag = self.go_to_pose(self.ur5_pose_1)
            if not flag:
                flag = self.go_to_pose(self.ur5_pose_1)

            # NAV HOME POSITION
            rospy.loginfo("STORE PICKUP HOME")
            self.go_to_named_pose("store_pickup_home_2")
            self.go_to_named_pose("store_pickup_home_2")

            self.go_to_named_pose("store_pickup_home_3_new")
            self.go_to_named_pose("store_pickup_home_3_new")

            # rospy.sleep(5)
            # self.avoid = 0

            self.moveit_goal_receiver.position.x = 1  # Picked the object signal to main
            self.moveit_special_receiver.position.x = 1
            rospy.loginfo("END STORE MOVEIT PICKING")
            self.moveit_goal_receiver.position.z = 0

        # Pantry Room
        if object_2b_picked == 'Coke' or object_2b_picked == 'Glass':
            self.go_to_named_pose("home_pantry_left")
            rospy.sleep(0.5)
            self.moveit_special_receiver.position.y = 1  # Image Window

            rospy.sleep(5)
            self.moveit_special_receiver.position.y = 0  # Image Window

            rospy.sleep(1)

            self.go_to_named_pose("home_pantry_right")
            rospy.sleep(0.5)
            self.moveit_special_receiver.position.y = 1  # Image Window
            rospy.sleep(5)
            self.moveit_special_receiver.position.y = 0  # Image Window

            # NAV HOME POSITION
            rospy.loginfo("NAV HOME")
            self.go_to_named_pose("home")
            self.go_to_named_pose("home")
            self.go_to_named_pose("home")

            rospy.sleep(1)

            self.moveit_goal_receiver.position.x = 1  # Picked the object signal to main
            self.moveit_special_receiver.position.x = 1
            rospy.loginfo("END PANTRY MOVEIT PICKING")

        # All Rooms
        if object_2b_picked == 'Glue' or object_2b_picked == 'Adhesive':
            rospy.loginfo("Meeting moveit started")
            rospy.sleep(1.)
            self.go_to_named_pose("allZeros2")
            rospy.sleep(2.)
            self.moveit_special_receiver.position.y = 1  # Image Window
            rospy.sleep(1.)
            self.place()

            object_detected_check = self.meet_detect(object_2b_picked)
            i = 2
            while i != 0:
                if object_detected_check == 1:
                    break
                rospy.sleep(1.)
                object_detected_check = self.meet_detect()
                i = i - 1

            if object_detected_check == 0:
                self.go_to_named_pose("allZeros2_new")
                rospy.sleep(1.)
                while not object_detected_check:
                    rospy.sleep(1.)
                    object_detected_check = self.meet_detect()

            rospy.sleep(2.)
            rospy.loginfo(self.meet_loc_ebot_base)
            self.moveit_special_receiver.position.y = 0  # Image Window

            q1 = quaternion_from_euler(-3.14, 0, math.pi)
            self.ur5_pose_1.orientation.x = q1[0]
            self.ur5_pose_1.orientation.y = q1[1]
            self.ur5_pose_1.orientation.z = q1[2]
            self.ur5_pose_1.orientation.w = q1[3]

             # FIRST POSITION
            rospy.loginfo("FIRST POSITION")
            self.ur5_pose_1.position.x = self.meet_loc_ebot_base[0]
            self.ur5_pose_1.position.y = self.meet_loc_ebot_base[1] - 0.18 - 0.1
            self.ur5_pose_1.position.z = self.meet_loc_ebot_base[2] + 0.12
            flag = self.go_to_pose(self.ur5_pose_1)
            flag = self.go_to_pose(self.ur5_pose_1)
            if not flag:
                flag = self.go_to_pose(self.ur5_pose_1)

            # GRABBING POSITION
            rospy.loginfo("GRABBING POSITION")
            # self.ur5_pose_1.position.x = self.coke[0] - 0.18
            self.ur5_pose_1.position.y = self.meet_loc_ebot_base[1] - 0.18
            self.ur5_pose_1.position.z = self.meet_loc_ebot_base[2] + 0.01
            flag = self.go_to_pose(self.ur5_pose_1)
            flag = self.go_to_pose(self.ur5_pose_1)
            if not flag:
                flag = self.go_to_pose(self.ur5_pose_1)

            # CLOSE & PICK
            rospy.loginfo("PICK")
            self.pick("close3")

            # MOVE AWAY
            rospy.loginfo("MOVE AWAY")
            # self.ur5_pose_1.position.x = self.coke[0] - 0.18 - 0.15
            self.ur5_pose_1.position.y = self.meet_loc_ebot_base[1] - 0.3  # -0.05 to compensate y dist.
            self.ur5_pose_1.position.z = self.meet_loc_ebot_base[2] + 0.2
            flag = self.go_to_pose(self.ur5_pose_1)
            flag = self.go_to_pose(self.ur5_pose_1)
            if not flag:
                flag = self.go_to_pose(self.ur5_pose_1)

            # NAV HOME POSITION
            rospy.loginfo("NAV HOME")
            self.go_to_named_pose("home")
            self.go_to_named_pose("home")

            rospy.sleep(1.)

            self.moveit_goal_receiver.position.x = 1  # Picked the object signal to main
            self.moveit_special_receiver.position.x = 1
            rospy.loginfo("END MEETING MOVEIT PICKING")

    def store_detect(self, data):
        rate1 = rospy.Rate(4)
        mi = 8
        check = 0
        if data == 'FPGA Board':
            obj_ids = [15, 40, 49, 64, 96]
            while mi != 0 and check == 0:
                for i in range(len(obj_ids)):
                    if self.t.frameExists("ebot_base") and self.t.frameExists("object_"+str(obj_ids[i])):
                        (self.store_loc_map, rotation) = self.t.lookupTransform("map", "object_"+str(obj_ids[i]), rospy.Time())
                        check = 1
                mi = mi - 1
                rate1.sleep()

        if data == 'Battery':
            obj_ids = [10, 33, 35, 42, 45, 50, 59, 75, 81, 103, 111, 114, 117]
            while mi != 0 and check == 0:
                for i in range(len(obj_ids)):
                    if self.t.frameExists("ebot_base") and self.t.frameExists("object_"+str(obj_ids[i])):
                        (self.store_loc_map, rotation) = self.t.lookupTransform("map", "object_"+str(obj_ids[i]), rospy.Time())
                        check = 1
                mi = mi - 1
                rate1.sleep()
        return check

    def meet_detect(self, data):
        rate1 = rospy.Rate(4)
        mi = 8
        check = 0
        if data == 'Adhesive':
            obj_ids = [26, 31, 32, 44, 46, 51, 57, 58, 62, 67, 69, 71, 80, 95, 98, 104, 105, 106, 107, 108, 109, 113]
            while mi != 0 and check == 0:
                for i in range(len(obj_ids)):
                    if self.t.frameExists("ebot_base") and self.t.frameExists("object_"+str(obj_ids[i])):
                        (self.meet_loc_ebot_base, rotation) = self.t.lookupTransform("ebot_base", "object_"+str(obj_ids[i]), rospy.Time(0))
                        check = 1
                mi = mi - 1
                rate1.sleep()

        if data == 'Glue':
            obj_ids = [11, 27, 28, 29, 30, 34, 37, 48, 52, 55, 65, 79, 84, 97, 99]
            while mi != 0 and check == 0:
                for i in range(len(obj_ids)):
                    if self.t.frameExists("ebot_base") and self.t.frameExists("object_"+str(obj_ids[i])):
                        (self.meet_loc_ebot_base, rotation) = self.t.lookupTransform("ebot_base", "object_"+str(obj_ids[i]), rospy.Time(0))
                        check = 1
                mi = mi - 1
                rate1.sleep()

        return check

    def coke_func(self):
        rate1 = rospy.Rate(4)
        mi = 8
        check = 0
        while mi != 0 and check == 0:
            if self.t.frameExists("ebot_base") and self.t.frameExists("object_16"):
                (self.coke, rotation) = self.t.lookupTransform("ebot_base", "object_16", rospy.Time())
                check = 1
            elif self.t.frameExists("ebot_base") and self.t.frameExists("object_14"):
                (self.coke, rotation) = self.t.lookupTransform("ebot_base", "object_14", rospy.Time())
                check = 1
            elif self.t.frameExists("ebot_base") and self.t.frameExists("object_13"):
                (self.coke, rotation) = self.t.lookupTransform("ebot_base", "object_13", rospy.Time())
                check = 1
            elif self.t.frameExists("ebot_base") and self.t.frameExists("object_17"):
                (self.coke, rotation) = self.t.lookupTransform("ebot_base", "object_17", rospy.Time())
                check = 1
            elif self.t.frameExists("ebot_base") and self.t.frameExists("object_18"):
                (self.coke, rotation) = self.t.lookupTransform("ebot_base", "object_18", rospy.Time())
                check = 1
            elif self.t.frameExists("ebot_base") and self.t.frameExists("object_19"):
                (self.coke, rotation) = self.t.lookupTransform("ebot_base", "object_19", rospy.Time())
                check = 1
            elif self.t.frameExists("ebot_base") and self.t.frameExists("object_20"):
                (self.coke, rotation) = self.t.lookupTransform("ebot_base", "object_20", rospy.Time())
                check = 1
            elif self.t.frameExists("ebot_base") and self.t.frameExists("object_21"):
                (self.coke, rotation) = self.t.lookupTransform("ebot_base", "object_21", rospy.Time())
                check = 1
            elif self.t.frameExists("ebot_base") and self.t.frameExists("object_22"):
                (self.coke, rotation) = self.t.lookupTransform("ebot_base", "object_22", rospy.Time())
                check = 1
            elif self.t.frameExists("ebot_base") and self.t.frameExists("object_23"):
                (self.coke, rotation) = self.t.lookupTransform("ebot_base", "object_23", rospy.Time())
                check = 1
            elif self.t.frameExists("ebot_base") and self.t.frameExists("object_24"):
                (self.coke, rotation) = self.t.lookupTransform("ebot_base", "object_24", rospy.Time())
                check = 1
            elif self.t.frameExists("ebot_base") and self.t.frameExists("object_25"):
                (self.coke, rotation) = self.t.lookupTransform("ebot_base", "object_25", rospy.Time())
                check = 1
            mi = mi - 1
            rate1.sleep()
        return check

    def go_to_pose(self, arg_pose):
        rospy.sleep(1.)
        self._group.set_start_state_to_current_state()
        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move
        return flag_plan

    def clear(self):
        self._group.clear_pose_targets()

    def go_to_named_pose(self, arg_pose):
        rospy.sleep(1.)
        self._group.set_start_state_to_current_state()
        self._group.set_named_target(arg_pose)
        self._group.go(wait=True)

    def pick(self, arg):
        self._hand_group.set_start_state_to_current_state()
        self._hand_group.set_named_target(arg)
        self._hand_group.go(wait=True)

    def place(self):
        self._hand_group.set_start_state_to_current_state()
        self._hand_group.set_named_target("open")
        self._hand_group.go(wait=True)

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()


def main():
    ur5 = Ur5Moveit()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        rate.sleep()


if __name__ == '__main__':
    main()
