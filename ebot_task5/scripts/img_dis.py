#!/usr/bin/env python
#from __future__ import print_function

#import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from PyQt5 import *
from geometry_msgs.msg import Pose

global img


class image_converter:

    def __init__(self):

        rospy.init_node('img_dis', anonymous=True)
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)

        self.window = 0
        self.close = 0
        self.close_one_more = 0
        self.glue_name_disp = 0
        self.battery_name_disp = 0
        self.adhesive_name_disp = 0
        self.fpga_name_disp = 0
        self.eyfi_name_disp = 0
        self.pair_of_wheels_name_disp = 0
        self.glass_name_disp = 0
        self.coke_name_disp = 0
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw2", Image, self.callback)
        self.detect_obj = rospy.Subscriber("/objects", Float32MultiArray, self.markobj)
        self.sub = rospy.Subscriber('/special_return', Pose, self.window_callback)
        global img
        # The object_ids for find_object_2d session
        self.battery_id = [10, 33, 35, 42, 45, 50, 59, 75, 81, 103, 111, 114, 117, 124, 127, 147]
        self.adhesive_id = [26, 31, 32, 44, 46, 51, 57, 58, 62, 67, 69, 71, 80, 95, 98, 104, 105, 106, 107, 108, 109, 113, 123, 125, 126, 128, 129, 149]
        self.fpga_id = [15, 40, 49, 64, 96, 116, 138]
        self.glue_id = [11, 27, 28, 29, 30, 34, 37, 48, 52, 55, 65, 79, 84, 97, 99, 112, 122, 137, 143, 146, 148]
        self.eyfi_id = [38, 47, 54, 56, 63, 66, 74, 78, 82, 94, 100, 139, 145, 150, 151]
        self.pair_of_wheels_id = [36, 39, 41, 43, 53, 60, 61, 68, 70, 72, 73, 76, 77, 83, 85, 101, 102, 110, 115, 118, 119, 120, 121, 140, 141, 142, 144]
        self.glass_id = [86, 87, 88, 90, 91, 92, 93]
        self.coke_id = [13, 14, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 89]

    def window_callback(self, data):# Used to know whether to display to not
        self.window = data.position.y

    def callback(self, data):#To get the image
        global img
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            img = self.cv_image
        except CvBridgeError as e:
            print(e)

        (rows, cols, channels) = self.cv_image.shape

        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def markobj(self, data):
        obj_array = data.data
        i = 0
        img_copy = self.cv_image.copy()
        if(len(obj_array)) and self.window == 1:
            self.close = 1
            self.close_one_more = 0
            while i < np.size(obj_array):  #To get the coordinates of bounding box from the array given by find_object_2d
                ido = int(obj_array[i])
                ow = obj_array[i+1]
                oh = obj_array[i+2]
                H = np.array([[obj_array[i+3],obj_array[i+6],obj_array[i+9]],
                     [obj_array[i+4],obj_array[i+7],obj_array[i+10]],
                     [obj_array[i+5],obj_array[i+8],obj_array[i+11]]])

                H_inv = np.linalg.inv(H)

                pts = np.float32([ [0,0],[0,oh],[ow,oh],[ow,0] ]).reshape(-1,1,2) #Gives the points for drawing Bounding Box
                dst = cv2.perspectiveTransform(pts,H)
                dst = np.int32(dst)

                cv2.polylines(img_copy, [dst], True, (255, 0, 0), 2) #Draws bounding box

                i += 12

                x = dst[0, 0][0]
                y = dst[0, 0][1] - 20
                tup = (x, y)
                if ido in self.glue_id: #Checks multiple object_ids for getting pose of object detected

                    if not self.glue_name_disp:
                        rospy.loginfo("Glue Identified")
                        self.glue_pos = tup
                        self.glue_name_disp = 1
                elif ido in self.battery_id:

                    if not self.battery_name_disp:
                        rospy.loginfo("Battery Identified")
                        self.battery_pos = tup
                        self.battery_name_disp = 1
                elif ido in self.adhesive_id:

                    if not self.adhesive_name_disp:
                        rospy.loginfo("Adhesive Identified")
                        x = dst[1, 0][0]
                        y = dst[1, 0][1] + 15
                        tup = (x, y)
                        self.adhesive_pos = tup
                        self.adhesive_name_disp = 1
                elif ido in self.fpga_id:

                    if not self.fpga_name_disp:
                        rospy.loginfo("FPGA Board Identified")
                        self.fpga_pos = tup
                        self.fpga_name_disp = 1
                elif ido in self.eyfi_id:

                    if not self.eyfi_name_disp:
                        rospy.loginfo("eYFI Board Identified")
                        self.eyfi_pos = tup
                        self.eyfi_name_disp = 1
                elif ido in self.pair_of_wheels_id:

                    if not self.pair_of_wheels_name_disp:
                        rospy.loginfo("Pair of Wheels Package Identified")
                        self.pair_of_wheels_pos = tup
                        self.pair_of_wheels_name_disp = 1

                elif ido in self.coke_id:

                    if not self.coke_name_disp:
                        rospy.loginfo("Coke Identified")
                        self.coke_pos = tup
                        self.coke_name_disp = 1

                elif ido in self.glass_id:

                    if not self.glass_name_disp:
                        rospy.loginfo("Glass Identified")
                        self.glass_pos = tup
                        self.glass_name_disp = 1



                # Based on objects detected print object names on bounding box
                if self.glue_name_disp == 1:
                    cv2.putText(img_copy, "Glue", self.glue_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                if self.battery_name_disp == 1:
                    cv2.putText(img_copy, "Battery", self.battery_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                if self.adhesive_name_disp == 1:
                    cv2.putText(img_copy, "Adhesive", self.adhesive_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                if self.fpga_name_disp == 1:
                    cv2.putText(img_copy, "FPGA Board", self.fpga_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                if self.eyfi_name_disp == 1:
                    cv2.putText(img_copy, "eYFI Board", self.eyfi_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                if self.pair_of_wheels_name_disp == 1:
                    cv2.putText(img_copy, "Pair of Wheels", self.pair_of_wheels_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                if self.coke_name_disp == 1:
                    cv2.putText(img_copy, "Coke", self.coke_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                if self.glass_name_disp == 1:
                    cv2.putText(img_copy, "Glass", self.glass_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                cv2.imshow("Image", img_copy)

        elif self.window == 0 and self.close == 1 and self.close_one_more == 0:
            self.glue_name_disp = 0
            self.battery_name_disp = 0
            self.adhesive_name_disp = 0
            self.fpga_name_disp = 0
            self.eyfi_name_disp = 0
            self.pair_of_wheels_name_disp = 0
            self.coke_name_disp = 0
            self.glass_name_disp = 0
            cv2.destroyAllWindows()
            self.close_one_more = 1
            self.close = 0



def main(args):
    ic = image_converter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print(" ")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
