#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose


class task5all():

    def __init__(self):

        rospy.init_node('mainc', anonymous=True)
        # rospy.Subscriber('/odom', Odometry, odom_callback)
        self.publisher = rospy.Publisher('/task5goal', Pose, queue_size=10)
        self.coordinates = Pose()
        
        while 1:
            self.nav_to_room(1, 1, 1)


    def nav_to_room(self, x, y, z):
    	self.coordinates.position.x = x
    	self.coordinates.position.y = y
    	self.coordinates.position.z = z
    	self.publisher.publish(self.coordinates)


if __name__ == '__main__':
    try:
        task5all()
    except rospy.ROSInterruptException:
        rospy.loginfo("UMMMM")
