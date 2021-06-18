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
value_angular = 0 

c = 0  # check for whether to send new goals or not
x = 0  # nav goal
y = 0
z = 0
z_check = 0  # Goal acquired by movebase
x_check = 0  # Goal reached
z_check_moveit = 0
x_check_moveit = 0
pantry_side_check_moveit = 0
o = quaternion_from_euler(0, 0, 0)
global coordinates, coordinates_moveit, source, obj_req
global publisher, publisher_moveit, publisher_drift, compensation, pan_drift, publisher_special, coordinates_special, pan_drift_2
global pan_drift_glue, pan_drift_glue_drop
drift_kill = False
drift_kill_new = False
pantry_object = 'Empty'
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
        compensation.angular.z = 4*error
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
    coordinates_moveit.header.frame_id = pantry_object
    coordinates_moveit.pose.pose.position.z = moveit_check
    # coordinates_moveit.pose.pose.position.y = moveit_proceed_with_picking
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
    global z_check_moveit, x_check_moveit, pantry_side_check_moveit
    z_check_moveit = data.position.z
    # x_check_moveit = data.position.x
    pantry_side_check_moveit = data.position.y

def special_return_callback(data):
    global x_check_moveit, dropped
    x_check_moveit = data.position.x
    dropped = data.position.z

def task5goal_received(data):
    global z_check, x_check
    z_check = data.position.z
    x_check = data.position.x


def pantryy():
    global x, y, z, o, pantry_object, moveit_check, value_angular, drift_kill, moveit_proceed_with_picking, pan_drift, drop, drop_room
    global drift_kill_new
    # p = [12.8, 1, 13, -0.7]  # pantry
    # x = p[0]
    # y = p[1]
    # z = 1
    # o = quaternion_from_euler(0, 0, -math.pi/2)
    # rospy.loginfo("FIRST PANTRY WAYPOINT")
    # while x_check == 0:
    #     pass
    # # nav_to_room()
    # rospy.loginfo("REACHED 1st")
    # x = p[2]
    # y = p[3]
    # z = 1
    # # o = quaternion_from_euler(0, 0, -math.pi/2)
    # rospy.sleep(2.)  # Wait till above goal reaches move base
    # while x_check == 0:
    #     pass
    # rospy.loginfo("REACHED 2nd")

    # value_angular = -1.57
    # drift_kill_new = False
    # pan_drift_new.start()

    # ti = 0
    # for i in source:
    #     if i == 'Pantry':
    #         pantry_object = obj_req[ti]
    #         source[ti] = 'Empty'
    #         break
    #     ti = ti + 1
    # moveit_check = 1
    # while z_check_moveit == 0:
    #     pass
    # moveit_check = 0
    # ktemp = 0
    # # drift_kill_new = True
    # # pan_drift_new.join()
    # while ktemp == 0:
    #     if pantry_side_check_moveit == 1:  # ie left
    #         ktemp = 1
    #         x = 13
    #         y = -0.7
    #         z = 1
    #         o = quaternion_from_euler(0, 0, 0)
    #         # rospy.sleep(2.)
    #     elif pantry_side_check_moveit == 2:  # ie right
    #         ktemp = 2
    #         x = 13
    #         y = -0.7
    #         z = 1
    #         o = quaternion_from_euler(0, 0, -3.14)
    # # rospy.sleep(1.)
    # while x_check == 0:
    #     pass
    # if ktemp == 1:
    #     x = 14.5
    #     y = -0.9
    #     z = 1
    #     o = quaternion_from_euler(0, 0, 0)
    #     rospy.loginfo("LEFT TABLE POINT GIVEN")
    #     value_angular = 0  # Store desired anular position and start drift correction
    # else:
    #     x = 11.7
    #     y = -1
    #     z = 1
    #     o = quaternion_from_euler(0, 0, -3.14)
    # rospy.sleep(1.)
    # while x_check == 0:
    #     pass
    # rospy.loginfo("Pantry Table Reached")

    # drift_kill = False
    # pan_drift.start()
    # rospy.loginfo("Drift Thread Started")

    # moveit_proceed_with_picking = 1   # Start picking up algo
    # rospy.loginfo("Sent proceed with picking")
    # while x_check_moveit == 0:
    #     rospy.loginfo("CHECKING x_check_moveit")
    #     rospy.loginfo(x_check_moveit)
    #     pass
    # moveit_proceed_with_picking = 0
    # drift_kill = True
    # pan_drift.join()
    # rospy.loginfo("Drift Thread Killed")
    # # rospy.sleep(0.5)

    # if ktemp == 1:  #Left
    #     o = quaternion_from_euler(0, 0, 3.05)
    #     z = 1
    # rospy.sleep(2.)
    # x = 13
    # y = -0.7
    # z = 1
    # while x_check == 0:
    #     pass
    # o = quaternion_from_euler(0, 0, 1.57)
    # rospy.sleep(2.)
    # x = 13
    # y = 1
    # z = 1
    # while x_check == 0:  # Reached pantry entry with object picked
    #     pass
    # z = 1
    # o = quaternion_from_euler(0, 0, 3.14)

#################################################
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
    # rospy.sleep(20.)
    # z = 0

    x = 7.6  # Meeting door dropbox
    y = 2.5
    o = quaternion_from_euler(0, 0, 0)
    z = 1
    rospy.sleep(3.)
    z = 0
    while x_check == 0:
        pass
##################################################################
    # rospy.loginfo("MEETING DROPBOX REACHED")

    # value_angular = 1.57
    # drift_kill = False
    # pan_drift_2.start()

    # drop_room = 'Meeting'
    # drop = 1

    # while dropped == 0:
    #     pass

    # drop = 0
    # rospy.loginfo("DROPPED")

    # drift_kill = True
    # pan_drift_2.join()

    # o = quaternion_from_euler(0, 0, 0)
    # z = 1
    # rospy.sleep(3.)

    # x = 7.6  # Meeti door PICK
    # y = 2.5
    # z = 1
    # while x_check == 0:
    #     pass

#############################################################
    # rospy.loginfo("MEETING DETECTION CAN START")


    #### Meeting Detection Starts from here
    value_angular = 0
    drift_kill = False
    pan_drift_glue.start()
    rospy.loginfo("Glue Drift Thread Started")
    # rospy.sleep(10.)

    ti = 0
    for i in source:
        if i == 'All':
            pantry_object = obj_req[ti]
            source[ti] = 'Empty'
            break
        ti = ti + 1
    moveit_check = 1
    while z_check_moveit == 0:
        pass
    moveit_check = 0

    while x_check_moveit == 0:
        rospy.loginfo("CHECKING x_check_moveit")
        rospy.loginfo(x_check_moveit)
        pass

    drift_kill = True
    pan_drift_glue.join()
    rospy.loginfo("Glue Drift Thread Stopped")
    rospy.loginfo("Glue Picked")

    x = 8.628
    y = 2.5
    o = quaternion_from_euler(0, 0, 0)
    z = 1
    rospy.sleep(2.)
    z = 0
    while x_check == 0:
        pass

    o = quaternion_from_euler(0, 0, -1.57)
    z = 1
    rospy.sleep(10.)
    z = 0

    x = 8.628  # Meeting room entry
    y = 1
    o = quaternion_from_euler(0, 0, 0)
    z = 1
    # rospy.loginfo("Meeting room entry given1")
    rospy.sleep(4.)
    z = 0

    # z = 1
    # # rospy.loginfo("Meeting room entry given2")
    # rospy.sleep(3.)
    # z = 0
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

    value_angular = 0
    drift_kill = False
    pan_drift_glue_drop.start()
    # rospy.loginfo("Glue Drop Drift Thread Started")
    rospy.sleep(5.)

    drop_room = 'Research'
    drop = 1

    while dropped == 0:
        pass

    drop = 0
    drift_kill = True
    pan_drift_glue_drop.join()
    # rospy.loginfo("Glue Drop Drift Thread Stopped")
    # rospy.loginfo("DROPPED")
    rospy.loginfo("Glue Dropped in Dropbox3")

    o = quaternion_from_euler(0, 0, -1.57)
    z = 1
    rospy.sleep(1.5)
    z = 0

    x = 10.650  # Research Room Drop
    y = 8.313
    z = 1
    rospy.sleep(3.)
    z = 0

    x = 25.247  # Store Room Location
    y = -2.721
    o = quaternion_from_euler(0, 0, -0.728)
    z = 1
    # rospy.loginfo("Store Room")
    rospy.sleep(2.)
    while x_check == 0:
        pass

    o = quaternion_from_euler(0, 0, 2.426)
    z = 1
    rospy.sleep(10.)
    z = 0

    x = 13  # Pantry Room Entry
    y = 1
    o = quaternion_from_euler(0, 0, -1.57)
    z = 1
    # rospy.loginfo("Pantry Room Entry")
    rospy.sleep(2.)
    while x_check == 0:
        pass

    x = 13  # Pantry Room Inside
    y = -0.7
    o = quaternion_from_euler(0, 0, -1.57)
    z = 1
    # rospy.loginfo("Pantry Room Inside")
    rospy.sleep(2.)
    while x_check == 0:
        pass

    o = quaternion_from_euler(0, 0, 1.57)
    z = 1
    rospy.sleep(10.)
    z = 0

    x = 13  # Pantry Room Entry
    y = 1
    # o = quaternion_from_euler(0, 0, 3.14)
    z = 1
    # rospy.loginfo("Pantry Room Entry Out")
    rospy.sleep(2.)
    while x_check == 0:
        pass

    x = 5.333  # Conference Room Entry
    y = 1
    o = quaternion_from_euler(0, 0, -1.57)
    z = 1
    # rospy.loginfo("Conference Room Entry")
    rospy.sleep(2.)
    while x_check == 0:
        pass

    x = 5.333  # Conference Room Inside
    y = -0.5
    o = quaternion_from_euler(0, 0, -1.57)
    z = 1
    # rospy.loginfo("Conference Room Inside")
    rospy.sleep(2.)
    while x_check == 0:
        pass

    o = quaternion_from_euler(0, 0, 1.57)
    z = 1
    rospy.sleep(10.)
    z = 0

    x = 5.333  # Conference Room Out
    y = 1
    o = quaternion_from_euler(0, 0, 3.14)
    z = 1
    # rospy.loginfo("Conference Room Out")
    rospy.sleep(2.)
    while x_check == 0:
        pass

    x = 0  # END
    y = 0
    o = quaternion_from_euler(0, 0, 3.14)
    z = 1
    # rospy.loginfo("END Position")
    rospy.sleep(2.)
    while x_check == 0:
        pass

def main():
    global coordinates, coordinates_moveit, moveit_check, z
    global publisher, publisher_moveit, publisher_drift, compensation, publisher_special, coordinates_special
    global c, source, obj_req, pan_drift, pan_drift_2, pan_drift_new, pan_drift_glue, pan_drift_glue_drop

    rospy.init_node('main', anonymous=True)
    pan = threading.Thread(target=pantryy)
    pan_drift = threading.Thread(target=drift)
    pan_drift_2 = threading.Thread(target=drift)
    pan_drift_glue = threading.Thread(target=drift)
    pan_drift_glue_drop = threading.Thread(target=drift)

    pan_drift_new = threading.Thread(target=drift_new)

    rospy.Subscriber('/odom', Odometry, odom_callback)
    publisher = rospy.Publisher('/task5goal', Pose, queue_size=10)
    rospy.Subscriber('/task5goal_received', Pose, task5goal_received)
    coordinates = Pose()

    rospy.Subscriber('/moveit_goal_received', Pose, moveit_goal_received)

    publisher_moveit = rospy.Publisher('/moveit_goal', Odometry, queue_size=10)
    coordinates_moveit = Odometry()

    publisher_drift = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    compensation = Twist()

    publisher_special = rospy.Publisher('/special', Odometry, queue_size=10)
    coordinates_special = Odometry()

    rospy.Subscriber('/special_return', Pose, special_return_callback)

    obj_req = ['FPGA Board', 'Coke', 'Glue']
    dest = ['Conference', 'Meeting', 'Research']
    n = len(dest)  # total destination
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

    while not rospy.is_shutdown():
        if c == 0:
            d_store = math.sqrt(((24-pose[0])**2)+((-1.736-pose[1])**2))
            d_pantry = math.sqrt(((13-pose[0])**2)+((1.115-pose[1])**2))
            # rospy.loginfo(d_pantry)
            # rospy.loginfo(d_store)
            if (d_pantry < d_store) and pantry != 0:
                # rospy.loginfo("SAEESH")
                # pantryy()
                pan.start()
                c = 1
                pantry = pantry - 1
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