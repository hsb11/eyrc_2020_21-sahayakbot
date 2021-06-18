#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import math
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import threading
from geometry_msgs.msg import Twist

pose = [0, 0]  # robot odom pose at any time
angular = 0  # angular z position at any time in rad
value_angular = 0  # desired angular position to be given to drift functions

c = 0  # check: after decision making, where to go
x = 0  # nav goals
y = 0
z = 0
z_check = 0  # Goal acquired by movebase
x_check = 0  # Goal reached by movebase
z_check_moveit = 0  # Goal acquired by moveit
x_check_moveit = 0  # Object picked/ dropped by moveit
side_check_moveit = 0  # Table left (1) or right (2) side
o = quaternion_from_euler(0, 0, 0)
global coordinates, coordinates_moveit, source, obj_req, dest
global publisher, publisher_moveit, publisher_drift, compensation, pan_drift, publisher_special, coordinates_special, pan_drift_2
global pan_drift_glue, pan_drift_glue_drop
global sto_drift, sto_drift_pick
global meet_drift
global research_drop_drift, conference_drop_drift
store_extra_home = 0

drift_kill = False
drift_kill_new = False
object_goal = 'Empty'  # Which object to be picked, to be given to moveit
moveit_check = 0  # Check to moveit script, when it should proceed
moveit_proceed_with_picking = 0
drop = 0  # Flag for moveit to drop
drop_room = 'Empty'  # Room in which it is dropping
dropped = 0  # Dropped indication from moveit


def drift_new():
    rate_drift = rospy.Rate(40)
    while 1:
        error = value_angular - angular
        compensation.angular.z = 10*error
        compensation.linear.x = 0
        publisher_drift.publish(compensation)
        if drift_kill_new:
            break
        rate_drift.sleep()


def drift():
    rate_drift = rospy.Rate(2)
    while 1:
        error = value_angular - angular
        compensation.angular.z = 7*error
        publisher_drift.publish(compensation)
        if drift_kill:
            break
        rate_drift.sleep()


def special_func():
    coordinates_special.header.frame_id = drop_room
    # coordinates_special.pose.pose.position.x = window
    coordinates_special.pose.pose.position.y = moveit_proceed_with_picking
    coordinates_special.pose.pose.position.z = drop
    publisher_special.publish(coordinates_special)


def moveit():
    coordinates_moveit.header.frame_id = object_goal
    coordinates_moveit.pose.pose.position.z = moveit_check
    coordinates_moveit.pose.pose.position.y = store_extra_home
    publisher_moveit.publish(coordinates_moveit)


def nav_to_room():
    coordinates.position.x = x
    coordinates.position.y = y
    coordinates.position.z = z
    coordinates.orientation.x = o[0]
    coordinates.orientation.y = o[1]
    coordinates.orientation.z = o[2]
    coordinates.orientation.w = o[3]
    publisher.publish(coordinates)


def odom_callback(data):
    global pose, angular
    pose = [data.pose.pose.position.x, data.pose.pose.position.y]
    explicit_quat = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    y = euler_from_quaternion(explicit_quat)
    angular = y[2]


def moveit_goal_received(data):
    global z_check_moveit, x_check_moveit, side_check_moveit
    z_check_moveit = data.position.z
    # x_check_moveit = data.position.x
    side_check_moveit = data.position.y

def special_return_callback(data):
    global x_check_moveit, dropped
    x_check_moveit = data.position.x
    dropped = data.position.z

def task5goal_received(data):
    global z_check, x_check
    z_check = data.position.z
    x_check = data.position.x


def drop_conference():
    global x, y, z, o, object_goal, moveit_check, value_angular, drift_kill, moveit_proceed_with_picking, pan_drift, drop, drop_room
    global drift_kill_new

    x = 8.628
    y = 2.5
    o = quaternion_from_euler(0, 0, -1.57)
    z = 1
    rospy.sleep(2.)
    while x_check == 0:
        pass

    x = 8.628
    y = 1
    o = quaternion_from_euler(0, 0, -3.14)
    z = 1
    rospy.sleep(2.)
    z = 0
    z = 1
    rospy.sleep(2.)
    z = 0
    while x_check == 0:
        pass

    x = 5.222  # Conference Room Drop Entry
    y = 1
    o = quaternion_from_euler(0, 0, -1.57)
    z = 1
    # rospy.loginfo("Conference room drop entry given")
    rospy.sleep(2.)
    while x_check == 0:
        pass

    x = 5.222  # Conference Room Drop Entry
    y = -0.5
    o = quaternion_from_euler(0, 0, -1.57)
    z = 1
    # rospy.loginfo("Conference room drop inside given")
    rospy.sleep(2.)
    while x_check == 0:
        pass
    rospy.loginfo("Conference Room Reached")
    rospy.sleep(3.)

    value_angular = -1.57
    drift_kill = False
    conference_drop_drift.start()
    # rospy.loginfo(object_goal + " Drop Drift Thread Started")
    rospy.sleep(2.)

    drop = 1

    while dropped == 0:
        pass

    drop = 0
    drift_kill = True
    conference_drop_drift.join()
    # rospy.loginfo(object_goal + " Drop Drift Thread Stopped")
    rospy.loginfo(object_goal + " Dropped in Dropbox1")

    o = quaternion_from_euler(0, 0, -2.4434)
    z = 1
    rospy.sleep(1.)
    z = 0

    while x_check == 0:
        pass

    o = quaternion_from_euler(0, 0, 3.14)
    z = 1
    rospy.sleep(1.)
    z = 0

    while x_check == 0:
        pass

    o = quaternion_from_euler(0, 0, 2.4434)
    z = 1
    rospy.sleep(1.)
    z = 0

    while x_check == 0:
        pass

    o = quaternion_from_euler(0, 0, 1.57)
    z = 1
    rospy.sleep(1.)
    z = 0

    while x_check == 0:
        pass

    x = 5.222  # Conference Room Drop Entry
    y = 1
    o = quaternion_from_euler(0, 0, 3.14)
    z = 1
    # rospy.loginfo("Conference room drop entry given AGAIN")
    rospy.sleep(2.)
    while x_check == 0:
        pass

    x = 0  # END
    y = 0
    o = quaternion_from_euler(0, 0, -3.12)
    z = 1
    rospy.sleep(2.)
    while x_check == 0:
        pass

    rospy.loginfo("Mission Accomplished!")

    while 1:
        pass


def drop_meeting():
    global x, y, z, o, object_goal, moveit_check, value_angular, drift_kill, moveit_proceed_with_picking, pan_drift, drop, drop_room
    global drift_kill_new

    x = 6.6  # Meeting Room Drop
    y = 2.5
    o = quaternion_from_euler(0, 0, 0)
    z = 1
    # rospy.loginfo("Meeting room drop given")
    rospy.sleep(2.)
    while x_check == 0:
        pass
    rospy.sleep(3.)

    drop = 1

    while dropped == 0:
        pass

    drop = 0
    # drift_kill = True
    # research_drop_drift.join()
    # rospy.loginfo(object_goal + " Drop Drift Thread Stopped")
    rospy.loginfo(object_goal + " Dropped in Dropbox2")
    rospy.sleep(0.5)


def drop_research():
    global x, y, z, o, object_goal, moveit_check, value_angular, drift_kill, moveit_proceed_with_picking, pan_drift, drop, drop_room
    global drift_kill_new

    x = 10.650  # Research Room Drop
    y = 6.313
    o = quaternion_from_euler(0, 0, 1.57)
    z = 1
    # rospy.loginfo("Research room drop given")
    rospy.sleep(2.)
    while x_check == 0:
        pass

    x = 10.650  # Research Room Drop
    y = 9.313
    o = quaternion_from_euler(0, 0, 0)
    z = 1
    # rospy.loginfo("Research room drop given")
    rospy.sleep(2.)
    while x_check == 0:
        pass

    rospy.loginfo("Research Lab Reached")

    value_angular = 0
    drift_kill = False
    research_drop_drift.start()
    # rospy.loginfo(object_goal + " Drop Drift Thread Started")
    rospy.sleep(5.)

    drop = 1

    while dropped == 0:
        pass

    drop = 0
    drift_kill = True
    research_drop_drift.join()
    # rospy.loginfo(object_goal + " Drop Drift Thread Stopped")
    rospy.loginfo(object_goal + " Dropped in Dropbox3")

    o = quaternion_from_euler(0, 0, -1.309)
    z = 1
    rospy.sleep(1.)
    z = 0

    while x_check == 0:
        pass

    o = quaternion_from_euler(0, 0, -1.57)
    z = 1
    rospy.sleep(1.)
    z = 0

    x = 10.850  # Research Room Drop
    y = 8.313
    z = 1
    rospy.sleep(2.)
    z = 0
    z = 1
    rospy.sleep(2.)
    z = 0
    z = 1
    rospy.sleep(2.)
    z = 0

    while x_check == 0:
        pass

    visit_all_rooms()


def meetingg():
    global x, y, z, o, object_goal, moveit_check, value_angular, drift_kill, moveit_proceed_with_picking, pan_drift, drop, drop_room
    global drift_kill_new
    ti = 0
    for i in source:
        if i == 'All':
            object_goal = obj_req[ti]  # Finding which object to be picked and set as goal
            source[ti] = 'Empty'
            drop_room = dest[ti]
            break
        ti = ti + 1

    # x = 8.628
    # y = 1
    # o = quaternion_from_euler(0, 0, 1.57)
    # z = 1
    # # rospy.loginfo("MEETING DOOR ENTRY")
    # while x_check == 0:
    #     pass

    # o = quaternion_from_euler(0, 0, 1.57)
    # z = 1
    # rospy.sleep(2.)

    # x = 8.628  # Meeting door inside
    # y = 2.5
    # o = quaternion_from_euler(0, 0, 1.57)
    # z = 1
    # while x_check == 0:
    #     pass

    # o = quaternion_from_euler(0, 0, 3.05)
    # z = 1
    # rospy.sleep(2.)

    # x = 6.6
    # y = 2.5
    # o = quaternion_from_euler(0, 0, 3.05)
    # z = 1
    # rospy.sleep(2.)
    # while x_check == 0:
    #     pass

    # o = quaternion_from_euler(0, 0, 0)
    # z = 1
    # rospy.sleep(15.)
    # z = 0

    # x = 7.6  # Meeting door dropbox
    # y = 2.5
    # o = quaternion_from_euler(0, 0, 0)
    # z = 1
    # rospy.sleep(3.)
    # z = 0
    # while x_check == 0:
    #     pass

    # rospy.loginfo("MEETING DETECTION CAN START")

    # #### Meeting Detection Starts from here
    # value_angular = 3.1416
    # drift_kill = False
    # meet_drift.start()
    # rospy.loginfo(object_goal + " Drift Thread Started")
    # rospy.sleep(3.)

    moveit_check = 1
    while z_check_moveit == 0:
        pass
    moveit_check = 0

    while x_check_moveit == 0:
        # rospy.loginfo("CHECKING x_check_moveit")
        # rospy.loginfo(x_check_moveit)
        pass

    # drift_kill = True
    # meet_drift.join()
    # rospy.loginfo(object_goal + " Drift Thread Stopped")
    rospy.loginfo(object_goal + " Picked")

    x = 4.2
    y = 5
    o = quaternion_from_euler(0, 0, -1.57)
    z = 1
    rospy.sleep(2.)
    z = 0
    while x_check == 0:
        pass

    # rospy.sleep(2.)

    x = 4.2
    y = 2.5
    o = quaternion_from_euler(0, 0, 0)
    z = 1
    rospy.sleep(2.)
    z = 0
    while x_check == 0:
        pass

    # o = quaternion_from_euler(0, 0, -1.57)
    # z = 1
    # rospy.sleep(10.)
    # z = 0

    # x = 8.628  # Meeting room entry
    # y = 1
    # o = quaternion_from_euler(0, 0, 0)
    # z = 1
    # rospy.loginfo("Meeting room entry given1")
    # rospy.sleep(4.)
    # z = 0

    # # z = 1
    # # # rospy.loginfo("Meeting room entry given2")
    # # rospy.sleep(3.)
    # # z = 0
    # while x_check == 0:
    #     pass

    if drop_room == 'Research':
        drop_research()
    elif drop_room == 'Conference':
        drop_conference()
    elif drop_room == 'Meeting':
        drop_meeting()
    # rospy.loginfo("C CHECK")


def storee():
    global x, y, z, o, object_goal, moveit_check, value_angular, drift_kill, moveit_proceed_with_picking, pan_drift, drop, drop_room
    global drift_kill_new, store_extra_home

    # ti = 0
    # for i in source:
    #     if i == 'Store':
    #         object_goal = obj_req[ti]  # Finding which object to be picked and set as goal
    #         source[ti] = 'Empty'
    #         drop_room = dest[ti]
    #         break
    #     ti = ti + 1

    # # # Now first traverse to store room detection point
    # x = 25.247
    # y = -2.721
    # o = quaternion_from_euler(0, 0, -0.6021)
    # z = 1
    # rospy.sleep(2.)
    # z = 0
    # while x_check == 0:
    #     pass
    # rospy.loginfo("Store Room Reached")

    # value_angular = -0.6291
    # drift_kill = False
    # sto_drift.start()
    # # rospy.loginfo("Store Det. Drift Working")
    # # rospy.sleep(1.)

    # moveit_check = 1
    # while z_check_moveit == 0:
    #     pass
    # moveit_check = 0

    # ktemp = 0
    # while side_check_moveit == 0:
    #     pass
    # ktemp = side_check_moveit
    # drift_kill = True
    # sto_drift.join()
    # # rospy.loginfo("Store Det. Drift Stopped")

    # o = quaternion_from_euler(0, 0, -2.1816)
    # z = 1
    # rospy.sleep(0.5)
    # z = 0

    # x = 24.63  # 24.3
    # y = -4.77  # -4
    # o = quaternion_from_euler(0, 0, -1.8326)  # -2.816
    # z = 1.
    # rospy.sleep(4.)
    # z = 0
    # z = 1
    # rospy.sleep(2.)
    # z = 0
    # # rospy.loginfo("Store Room Turn1 point GIVEN")
    # while x_check == 0:
    #     pass
    # # rospy.loginfo("Store Room Turn1 point reached")

    # o = quaternion_from_euler(0, 0, 1.02)
    # z = 1
    # rospy.sleep(2)
    # z = 0

    # if ktemp == 1:  # ie left
    #     x = 26
    #     y = -3.185
    #     o = quaternion_from_euler(0, 0, 0.928)
    #     z = 1
    # elif ktemp == 2:  # ie right
    #     x = 25.60  # 25.679
    #     y = -3.64  # -3.695
    #     o = quaternion_from_euler(0, 0, 0.928)
    #     z = 1
    # rospy.sleep(4.)
    # z = 0
    # z = 1
    # rospy.sleep(2.)
    # z = 0
    # z = 1
    # rospy.sleep(2.)
    # z = 0
    # while x_check == 0:
    #     pass
    # # rospy.loginfo("Store Room Pick Up point reached")

    # rospy.sleep(12.)

    # value_angular = 0.928
    # drift_kill_new = False
    # sto_drift_pick.start()
    # # rospy.loginfo("Store Pickup Drift Working")
    # rospy.sleep(5.)

    # moveit_proceed_with_picking = 1   # Start picking up algo
    # # rospy.loginfo("Sent proceed with picking")
    # while x_check_moveit == 0:
    #     # rospy.loginfo("CHECKING x_check_moveit")
    #     # rospy.loginfo(x_check_moveit)
    #     pass
    # moveit_proceed_with_picking = 0
    # rospy.loginfo(object_goal + " Picked")
    # drift_kill_new = True
    # sto_drift_pick.join()
    # # rospy.loginfo("Store Pickup Drift Thread Killed")
    # rospy.sleep(0.5)

    # x = 25.53
    # y = -2.21
    # o = quaternion_from_euler(0, 0, 2.5)
    # z = 1
    # rospy.sleep(2.)
    # z = 0
    # while x_check == 0:
    #     pass

    # if drop_room == 'Research':
    #     drop_research()
    # elif drop_room == 'Conference':
    #     drop_conference()
    # rospy.loginfo("C CHECK")
    visit_all_rooms()


def visit_all_rooms():
    global x, y, z, o, object_goal, moveit_check, value_angular, drift_kill, moveit_proceed_with_picking, pan_drift, drop, drop_room
    global drift_kill_new

    # object_goal = 'Coke'

    # x = 13  # Pantry Room Entry
    # y = 1
    # o = quaternion_from_euler(0, 0, -1.57)
    # z = 1
    # # rospy.loginfo("Pantry Room Entry")
    # rospy.sleep(2.)
    # while x_check == 0:
    #     pass

    # x = 13  # Pantry Room Inside
    # y = -0.7
    # o = quaternion_from_euler(0, 0, -1.57)
    # z = 1
    # rospy.sleep(2.)
    # while x_check == 0:
    #     pass
    # rospy.loginfo("Pantry Room Reached")

    # rospy.sleep(4)
    # moveit_check = 1
    # while z_check_moveit == 0:
    #     pass

    # moveit_check = 0
    # while x_check_moveit == 0:
    #     pass

    # o = quaternion_from_euler(0, 0, -3.14)
    # z = 1
    # rospy.sleep(4.)
    # z = 0

    # o = quaternion_from_euler(0, 0, 2.35)
    # z = 1
    # rospy.sleep(4.)
    # z = 0
    # z = 1
    # rospy.sleep(0.5)
    # z = 0

    # o = quaternion_from_euler(0, 0, 1.57)
    # z = 1
    # rospy.sleep(4.)
    # z = 0
    # z = 1
    # rospy.sleep(0.5)
    # z = 0

    # x = 13  # Pantry Room Entry
    # y = 1
    # # o = quaternion_from_euler(0, 0, 3.14)
    # z = 1
    # # rospy.loginfo("Pantry Room Entry Out")
    # rospy.sleep(2.)
    # while x_check == 0:
    #     pass

    # x = 8.628
    # y = 1
    # o = quaternion_from_euler(0, 0, 1.57)
    # z = 1
    # # rospy.loginfo("MEETING DOOR ENTRY")
    # while x_check == 0:
    #     pass

    # x = 8.728  # Meeting door inside
    # y = 2.5
    # o = quaternion_from_euler(0, 0, 0.9599)
    # z = 1
    # rospy.sleep(2.)
    # z = 0
    # z = 1
    # rospy.sleep(2.)
    # z = 0
    # while x_check == 0:
    #     pass

    # x = 9.128  # Meeting door MORE inside
    # y = 4.8
    # o = quaternion_from_euler(0, 0, -3.14)
    # z = 1
    # rospy.sleep(2.)
    # z = 0
    # z = 1
    # rospy.sleep(2.)
    # z = 0
    # while x_check == 0:
    #     pass

    # rospy.loginfo("Meeting Room Reached")

    # x = 5.85   # Meeting door BONUS PICKUP POINT
    # y = 5
    # o = quaternion_from_euler(0, 0, -3.14)
    # z = 1
    # rospy.sleep(2.)
    # z = 0
    # z = 1
    # rospy.sleep(2.)
    # z = 0
    # while x_check == 0:
    #     pass

    # rospy.sleep(4.)
    meetingg()

    x = 8.628  # Meeting door inside AGAIN
    y = 2.5
    o = quaternion_from_euler(0, 0, 0.7854)
    z = 1
    rospy.sleep(2.)
    z = 0
    z = 1
    rospy.sleep(2.)
    z = 0
    while x_check == 0:
        pass

    x = 9.128  # Meeting door MORE inside
    y = 5
    o = quaternion_from_euler(0, 0, -3.14)
    z = 1
    rospy.sleep(2.)
    z = 0
    z = 1
    rospy.sleep(2.)
    z = 0
    while x_check == 0:
        pass

    x = 5.85   # Meeting door BONUS PICKUP POINT AGAIN
    y = 5
    o = quaternion_from_euler(0, 0, 3.1416)
    z = 1
    rospy.sleep(2.)
    z = 0
    z = 1
    rospy.sleep(2.)
    z = 0
    while x_check == 0:
        pass

    rospy.sleep(4.)
    meetingg()

    # x = 5.222  # Conference Room Entry
    # y = 1
    # o = quaternion_from_euler(0, 0, -1.57)
    # z = 1
    # # rospy.loginfo("Conference Room Entry")
    # rospy.sleep(2.)
    # while x_check == 0:
    #     pass

    # x = 5.222  # Conference Room Inside
    # y = -0.5
    # o = quaternion_from_euler(0, 0, -1.57)
    # z = 1
    # rospy.sleep(2.)
    # while x_check == 0:
    #     pass
    # rospy.loginfo("Conference Room Reached")

    # rospy.sleep(1.)

    # o = quaternion_from_euler(0, 0, 1.57)
    # z = 1
    # rospy.sleep(10.)
    # z = 0

    # x = 5.222  # Conference Room Out
    # y = 1
    # o = quaternion_from_euler(0, 0, 3.14)
    # z = 1
    # # rospy.loginfo("Conference Room Out")
    # rospy.sleep(2.)
    # while x_check == 0:
    #     pass


def main():
    global coordinates, coordinates_moveit, moveit_check, z
    global publisher, publisher_moveit, publisher_drift, compensation, publisher_special, coordinates_special
    global c, source, obj_req, pan_drift, pan_drift_2, pan_drift_new, pan_drift_glue, pan_drift_glue_drop, dest
    global sto_drift, sto_drift_pick, meet_drift, research_drop_drift, conference_drop_drift

    rospy.init_node('main2_task6', anonymous=True)
    # pan = threading.Thread(target=pantryy)  # Pantry function thread instance
    sto = threading.Thread(target=storee)  # Store function thread instance
    meet = threading.Thread(target=meetingg)  # Meeting function thread instance

    # pan_drift = threading.Thread(target=drift)
    # pan_drift_2 = threading.Thread(target=drift)
    # pan_drift_glue = threading.Thread(target=drift)
    # pan_drift_glue_drop = threading.Thread(target=drift)

    # pan_drift_new = threading.Thread(target=drift_new)

    sto_drift = threading.Thread(target=drift)
    sto_drift_pick = threading.Thread(target=drift_new)

    meet_drift = threading.Thread(target=drift)

    research_drop_drift = threading.Thread(target=drift)
    conference_drop_drift = threading.Thread(target=drift)

    rospy.Subscriber('/odom', Odometry, odom_callback)

    publisher = rospy.Publisher('/task5goal', Pose, queue_size=10)
    coordinates = Pose()
    rospy.Subscriber('/task5goal_received', Pose, task5goal_received)

    rospy.Subscriber('/moveit_goal_received', Pose, moveit_goal_received)
    publisher_moveit = rospy.Publisher('/moveit_goal', Odometry, queue_size=10)
    coordinates_moveit = Odometry()

    publisher_drift = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    compensation = Twist()

    publisher_special = rospy.Publisher('/special', Odometry, queue_size=10)
    coordinates_special = Odometry()
    rospy.Subscriber('/special_return', Pose, special_return_callback)

    obj_req = ['Battery', 'Glue', 'Adhesive']
    dest = ['Research', 'Meeting', 'Conference']
    n = len(dest)  # total objects req
    source = [0]*n  # Source room for each object array
    si = 0  # index for source
    store = 0  # no of objects to be picked from store
    pantry = 0  # from pantry
    allrooms = 0  # from all rooms

    for i in obj_req:
        if i == 'FPGA Board' or i == 'eYFI Board' or i == 'Pair of Wheels Package' or i == 'Battery':
            source[si] = 'Store'
            store = store + 1
        elif i == 'Coke' or i == 'Glass':
            source[si] = 'Pantry'
            pantry = pantry + 1
        else:
            source[si] = 'All'
            allrooms = allrooms + 1
        si = si + 1

    # rospy.loginfo(o1)
    rate = rospy.Rate(40)
    # rospy.sleep(2.)
    while not rospy.is_shutdown():
        if c == 0:
            # rospy.loginfo(pose)
            d_store = math.sqrt(((25.247-pose[0])**2)+((-2.721-pose[1])**2))
            d_pantry = math.sqrt(((13-pose[0])**2)+((1.115-pose[1])**2))
            d_meeting = math.sqrt(((8.628-pose[0])**2)+((1-pose[1])**2))

            # rospy.loginfo(d_pantry)
            # rospy.loginfo(d_store)
            # rospy.loginfo(d_meeting)
            sto.start()
            c = 1
            store = store - 1
            # if min(d_pantry, d_store, d_meeting) == d_meeting and allrooms != 0:
            #     meet.start()
            #     c = 1
            #     allrooms = allrooms - 1
            # elif (d_pantry < d_store) and pantry != 0:
            #     # rospy.loginfo("SAEESH")
            #     # pantryy()
            #     # pan.start()
            #     c = 1
            #     pantry = pantry - 1
            # elif (d_pantry > d_store) and store != 0:
            #     sto.start()
            #     c = 1
            #     store = store - 1
            # elif store != 0 and pantry == 0:
            #     sto.start()
            #     c = 1
            #     store = store - 1
        else:
            if z_check == 1:
                z = 0
            if z_check_moveit == 1:
                moveit_check = 0
            nav_to_room()
        # rospy.loginfo('HI')
        moveit()
        special_func()
        rate.sleep()

    # rospy.spin()

# Python Main
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
