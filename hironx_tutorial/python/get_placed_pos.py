#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from jsk_recognition_msgs.msg import LineArray
from geometry_msgs.msg import Point

import cv2
import numpy as np
from copy import deepcopy

TH = 50
X_OFFSET = -5
Y_OFFSET = -15
EXTENTION = 0

class VisualFeedback:

    def __init__(self):
        self.before_img = None
        self.after_img = None
        self.output_img = None
        self.count = 0 #for visualize
        self.flag = False
        self.ltop_x = None; self.ltop_y = None; self.lbottom_x = None; self.lbottom_y = None
        self.rtop_x = None; self.rtop_y = None; self.rbottom_x = None; self.rbottom_y = None
        self.bridge = CvBridge()

        self.pub = rospy.Publisher("/lunchbox_positions/output", Point, queue_size=1)
        rospy.Subscriber("/before_place_img", Image, self.bimg_cb)
        rospy.Subscriber("/placed_img", Image, self.aimg_cb)
        rospy.Subscriber("/lunchbox_positions/input", LineArray, self.position_cb)

    def bimg_cb(self, msg):
        # print("Called before img")
        if not self.flag:
            self.before_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imwrite("/home/tanemoto/Desktop/images/before.png", self.before_img)
            self.flag = True
            self.count += 1

    def aimg_cb(self, msg):
        self.after_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite("/home/tanemoto/Desktop/images/after.png", self.after_img)
        if self.ltop_x and self.flag:
            self.flag = False
            print("Called before_img and after_img and positions")
            self.img_processing()

    def position_cb(self, msg):
        if not self.ltop_x:
            print("Called lunchbox position")
        linfo = msg.lines[0]
        rinfo = msg.lines[1]
        self.ltop_x = linfo.x1; self.ltop_y = linfo.y1; self.lbottom_x = linfo.x2; self.lbottom_y = linfo.y2
        self.rtop_x = rinfo.x1; self.rtop_y = rinfo.y1; self.rbottom_x = rinfo.x2; self.rbottom_y = rinfo.y2
        self.ltop_x += X_OFFSET; self.lbottom_x += X_OFFSET; self.rtop_x += X_OFFSET; self.rbottom_x += X_OFFSET
        self.ltop_y += Y_OFFSET; self.lbottom_y += Y_OFFSET; self.rtop_y += Y_OFFSET; self.rbottom_y += Y_OFFSET

    def get_food_pos(self, before_img, after_img):
        im_diff = before_img.astype(int) - after_img.astype(int)
        im_diff_abs = np.abs(im_diff)
        im_diff_img = im_diff_abs.astype(np.uint8)
        im_diff_img[np.where(im_diff_abs[:,:,0] < TH) and np.where(im_diff_abs[:,:,1] < TH) and np.where(im_diff_abs[:,:,2] < TH)] = [0, 0, 0]
        img_gray = cv2.cvtColor(im_diff_img, cv2.COLOR_BGR2GRAY)
        _, img_binary = cv2.threshold(img_gray, 1, 255, cv2.THRESH_BINARY)
        # remove noise
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
        img_del_noise = cv2.morphologyEx(img_binary, cv2.MORPH_OPEN, kernel)
        cv2.imwrite("/home/tanemoto/Desktop/images/diff.png", img_del_noise)
        where = np.where(img_del_noise != 0)
        if len(where[0]) < 50:
            return None, None
        pos_x = (np.min(where[1]) + np.max(where[1])) / 2
        pos_y = (np.min(where[0]) + np.max(where[0])) / 2
        return pos_x, pos_y
        
    def img_processing(self):
        self.output_img = deepcopy(self.after_img)
        cv2.line(self.output_img, (int(self.rtop_x), int(self.rtop_y)), (int(self.rbottom_x), int(self.rbottom_y)), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
        cv2.line(self.output_img, (int(self.ltop_x), int(self.ltop_y)), (int(self.rtop_x), int(self.rtop_y)), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
        cv2.line(self.output_img, (int(self.ltop_x), int(self.ltop_y)), (int(self.lbottom_x), int(self.lbottom_y)), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
        cv2.line(self.output_img, (int(self.lbottom_x), int(self.lbottom_y)), (int(self.rbottom_x), int(self.rbottom_y)), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
        top = int((self.ltop_y + self.rtop_y) / 2 - EXTENTION)
        bottom = int((self.lbottom_y + self.rbottom_y) / 2 + EXTENTION)
        left = int((self.ltop_x + self.lbottom_x) / 2 - EXTENTION)
        right = int((self.rtop_x + self.rbottom_x) / 2 + EXTENTION)
        lbox_bimg = self.before_img[top: bottom, left: right, :]
        lbox_aimg = self.after_img[top: bottom, left: right, :]
        pos_x, pos_y = self.get_food_pos(lbox_bimg, lbox_aimg)
        if pos_x:
            pos_x += left
            pos_y += top
        else:
            pos_x = 0
            pos_y = 0
        cv2.drawMarker(self.output_img, (int(pos_x), int(pos_y)), (0, 0, 255), markerType=cv2.MARKER_STAR, markerSize=20)
        cv2.imwrite("/home/tanemoto/Desktop/images/output_" + str(self.count) + ".png", self.output_img)
        # publish
        if pos_x:
            pos_x -= (left + right) / 2
            pos_y -= (top + bottom) / 2
        pub_msg = Point()
        pub_msg.x = pos_x
        pub_msg.y = pos_y
        pub_msg.z = 0
        self.pub.publish(pub_msg)

if __name__ == '__main__':
    rospy.init_node("get_placed_pos")
    vis = VisualFeedback()
    rospy.spin()

