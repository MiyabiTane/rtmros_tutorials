#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from jsk_recognition_msgs.msg import LineArray
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

import cv2
import numpy as np
from copy import deepcopy

TH = 50


class VisualFeedback:

    def __init__(self):
        self.before_img = None
        self.after_img = None
        self.output_img = None
        self.number = 0
        self.bridge = CvBridge()

        self.pub = rospy.Publisher("/lunchbox_positions/output", Point, queue_size=1)
        rospy.Subscriber("/place_flag", Bool, self.img_cb)
        rospy.Subscriber("/lunchbox_positions/input", LineArray, self.position_cb)

    def img_cb(self, msg):
        # flag > 0 -> before placing food : number of food
        # flag < 0 -> after placing food
        self.number = msg.data
        if msg.data > 0:
            img_msg = rospy.wait_for_message("/head_camera/rgb/image_raw", Image)
            self.before_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        elif msg.data < 0:
            img_msg = rospy.wait_for_message("/head_camera/rgb/image_raw", Image)
            self.after_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")

    def get_food_pos(self, before_img, after_img):
        im_diff = before_img.astype(int) - after_img.astype(int)
        im_diff_abs = np.abs(im_diff)
        im_diff_abs[np.where(im_diff_abs[:,:,0] < TH) and np.where(im_diff_abs[:,:,1] < TH) and np.where(im_diff_abs[:,:,2] < TH)] = [0, 0, 0]
        # cv2.imwrite("/home/tork/Downloads/diff_.png", im_diff_abs)
        where = np.where(im_diff_abs != [0, 0, 0])
        pos_x = (np.min(where[1]) + np.max(where[1])) / 2
        pos_y = (np.min(where[0]) + np.max(where[0])) / 2
        return pos_x, pos_y

    def position_cb(self, msg):
        if self.before_img and self.after_img:
            linfo = msg.lines[0]
            rinfo = msg.lines[1]
            ltop_x = linfo.x1; ltop_y = linfo.y1; lbottom_x = linfo.x2; lbottom_y = linfo.y2
            rtop_x = rinfo.x1; rtop_y = rinfo.y1; rbottom_x = rinfo.x2; rbottom_y = rinfo.y2
            self.output_img = deepcopy(self.after_img)
            cv2.line(self.output_img, (int(rtop_x), int(rtop_y)), (int(rbottom_x), int(rbottom_y)), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
            cv2.line(self.output_img, (int(ltop_x), int(ltop_y)), (int(rtop_x), int(rtop_y)), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
            cv2.line(self.output_img, (int(ltop_x), int(ltop_y)), (int(lbottom_x), int(lbottom_y)), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
            cv2.line(self.output_img, (int(lbottom_x), int(lbottom_y)), (int(rbottom_x), int(rbottom_y)), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
            top = int((ltop_y + rtop_y) / 2 - 20)
            bottom = int((lbottom_y + rbottom_y) / 2 + 20)
            left = int((ltop_x + lbottom_x) / 2 - 20)
            right = int((rtop_x + rbottom_x) / 2 + 20)
            lbox_bimg = self.before_img[top: bottom, left: right, :]
            lbox_aimg = self.after_img[top: bottom, left: right, :]
            pos_x, pos_y = self.get_food_pos(lbox_bimg, lbox_aimg)
            pos_x += left
            pos_y += top
            cv2.drawMarker(self.output_img, (int(pos_x), int(pos_y)), (0, 0, 255), markerType=cv2.MARKER_STAR, markerSize=10)
            cv2.imwrite("/home/tanemoto/Desktop/images/output_" + str(self.number) + ".png", self.output_img)
            # publish
            pub_msg = Point()
            pub_msg.x = pos_x
            pub_msg.y = pos_y
            pub_msg.z = 0
            self.pub.publish(pub_msg)
            self.before_img = None
            self.after_img = None

if __name__ == '__main__':
    rospy.init_node("get_placed_pos")
    vis = VisualFeedback()
    rospy.spin()

