#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
from hironx_tutorial.msg import FoodPacking
from hironx_tutorial.msg import LunchBoxStatus

import cv2
import numpy as np
from copy import deepcopy

TH = 50
X_OFFSET = -5
Y_OFFSET = -15
EXTENTION = 0
TH2 = 15
TH3 = 10

class VisualFeedback:

    def __init__(self):
        self.before_img = None
        self.after_img = None
        self.output_img = None
        self.empty_img = None
        self.stamp = None
        self.pub_msg = None
        self.count = 0 #for visualize
        self.lt_x = None; self.lt_y = None; self.lb_x = None; self.lb_y = None
        self.rt_x = None; self.rt_y = None; self.rb_x = None; self.rb_y = None
        self.bridge = CvBridge()

        self.pub = rospy.Publisher("~output", FoodPacking, queue_size=1)
        rospy.Subscriber("~input", LunchBoxStatus, self.status_cb)

    def status_cb(self, msg):
        flag = False
        if not self.stamp:
            flag = True
        elif self.stamp != msg.header.stamp:
            flag = True
        if flag:
            print("Called Lunchbox status")
            self.stamp = msg.header.stamp
            self.empty_img = self.bridge.imgmsg_to_cv2(msg.empty, desired_encoding="bgr8")
            self.before_img = self.bridge.imgmsg_to_cv2(msg.before, desired_encoding="bgr8")
            self.after_img = self.bridge.imgmsg_to_cv2(msg.after, desired_encoding="bgr8")
            self.lt_x = msg.ltop.x; self.lt_y = msg.ltop.y; self.lb_x = msg.lbottom.x; self.lb_y = msg.lbottom.y
            self.rt_x = msg.rtop.x; self.rt_y = msg.rtop.y; self.rb_x = msg.rbottom.x; self.rb_y = msg.rbottom.y
            self.lt_x += X_OFFSET; self.lb_x += X_OFFSET; self.rt_x += X_OFFSET; self.rb_x += X_OFFSET
            self.lt_y += Y_OFFSET; self.lb_y += Y_OFFSET; self.rt_y += Y_OFFSET; self.rb_y += Y_OFFSET
            self.publish_info()
            self.count += 1
        if self.pub_msg:
            self.pub.publish(self.pub_msg)
            
            
    def get_diff_img(self, before_img, after_img, k_size=3):
        im_diff = before_img.astype(int) - after_img.astype(int)
        im_diff_abs = np.abs(im_diff)
        im_diff_img = im_diff_abs.astype(np.uint8)
        im_diff_img[np.where(im_diff_abs[:,:,0] < TH) and np.where(im_diff_abs[:,:,1] < TH) and np.where(im_diff_abs[:,:,2] < TH)] = [0, 0, 0]
        img_gray = cv2.cvtColor(im_diff_img, cv2.COLOR_BGR2GRAY)
        _, img_binary = cv2.threshold(img_gray, 1, 255, cv2.THRESH_BINARY)
        # remove noise
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(k_size,k_size))
        img_del_noise = cv2.morphologyEx(img_binary, cv2.MORPH_OPEN, kernel)
        return img_del_noise

    def get_food_info(self, before_img, after_img):
        diff_img = self.get_diff_img(before_img, after_img)
        where = np.where(diff_img != 0)
        boxes = []
        if len(where[0]) < 50:
            return None, None, None, None
        _, contours, _hierarchy = cv2.findContours(diff_img ,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        max_size = 0
        box = None
        if len(contours) == 0:
            return None, None, None, None
        for cnt in contours:
            if max_size < cv2.contourArea(cnt):
                max_size = cv2.contourArea(cnt)
                # rect = cv2.minAreaRect(cnt)
                # box = cv2.boxPoints(rect)
                # box = np.int0(box)
                x, y, w, h = cv2.boundingRect(cnt)
        # print("BOX: {}".format(box))
        cv2.imwrite("/home/tanemoto/Desktop/images/diff_" + str(self.count) + ".png", diff_img)
        # vis_img = cv2.drawContours(after_img, [box], 0, (0,0,255), 2)
        vis_img = cv2.rectangle(after_img,(x,y),(x+w,y+h),(0,0,255),2)
        cv2.imwrite("/home/tanemoto/Desktop/images/diff_box_" + str(self.count) + ".png", vis_img)
        # pos_x = np.mean(box[:, 0])
        # pos_y = np.mean(box[:, 1])
        # width = np.max(box[:, 0]) - np.min(box[:, 0])
        # height = np.max(box[:, 1]) - np.min(box[:, 1])
        pos_x = x + w / 2
        pos_y = y + h / 2
        return pos_x, pos_y, w, h

    def if_can_place(self, diff_img):
        image_size = diff_img.size
        whitePixels = cv2.countNonZero(diff_img)
        # print("full : {}%".format(float(whitePixels) / image_size * 100))
        if float(whitePixels) / image_size > 0.4:
            return False
        return True

    def get_available_angle(self, empty_img, before_img, pos, size):
        # pos:Tuple[x: float, y: float], size:Tuple[width: float, height: float]
        ans_str = ""
        # check availability of lunchbox
        diff_img =  self.get_diff_img(empty_img, before_img)
        # calculate how to approach placed food
        H, W, _C = empty_img.shape
        x_ = pos[0]
        y_ = pos[1]
        width_ = size[0]
        height_ = size[1]
        # check top space
        if y_ - height_ / 2 - TH2 >= 0:
            top = int(y_ - height_ / 2 - TH3)
            bottom = int(y_ - height_ / 2)
            left = int(max(0, x_ - width_ / 2))
            right = int(min(W, x_ + width_ / 2))
            ans = self.if_can_place(diff_img[top: bottom, left: right])
            cv2.imwrite("/home/tanemoto/Desktop/images/diff_" + str(self.count) + "_top.png", diff_img[top: bottom, left: right])
            if ans:
                ans_str += "top,"
        # check bottom space
        if y_ + height_ / 2 + TH2 <= H:
            top = int(y_ + height_ / 2)
            bottom = int(y_ + height_ / 2 + TH3)
            left = int(max(0, x_ -width_ / 2))
            right = int(min(W, x_ + width_ / 2))
            ans = self.if_can_place(diff_img[top: bottom, left: right])
            cv2.imwrite("/home/tanemoto/Desktop/images/diff_" + str(self.count) + "_bottom.png", diff_img[top: bottom, left: right])
            if ans:
                ans_str += "bottom,"
        # check left space
        if x_ - width_ / 2 - TH2 >= 0:
            top = int(max(0, y_ - height_ / 2))
            bottom = int(min(H, y_ + height_ / 2))
            left = int(x_ - width_ / 2 - TH3)
            right = int(x_ - width_ / 2)
            ans = self.if_can_place(diff_img[top: bottom, left: right])
            cv2.imwrite("/home/tanemoto/Desktop/images/diff_" + str(self.count) + "_left.png", diff_img[top: bottom, left: right])
            if ans:
                ans_str += "left,"
        # check right space
        if x_ + width_ / 2 + TH2 <= W:
            top = int(max(0, y_ - height_ / 2))
            bottom = int(min(H, y_ + height_ / 2))
            left = int(x_ + width_ / 2)
            right = int(x_ + width_ / 2 + TH3)
            ans = self.if_can_place(diff_img[top: bottom, left: right])
            cv2.imwrite("/home/tanemoto/Desktop/images/diff_" + str(self.count) + "_right.png", diff_img[top: bottom, left: right])
            if ans:
                ans_str += "right"
        return ans_str
        
    def publish_info(self):
        self.output_img = deepcopy(self.after_img)
        cv2.line(self.output_img, (int(self.rt_x), int(self.rt_y)), (int(self.rb_x), int(self.rb_y)), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
        cv2.line(self.output_img, (int(self.lt_x), int(self.lt_y)), (int(self.rt_x), int(self.rt_y)), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
        cv2.line(self.output_img, (int(self.lt_x), int(self.lt_y)), (int(self.lb_x), int(self.lb_y)), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
        cv2.line(self.output_img, (int(self.lb_x), int(self.lb_y)), (int(self.rb_x), int(self.rb_y)), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
        top = int((self.lt_y + self.rt_y) / 2 - EXTENTION)
        bottom = int((self.lb_y + self.rb_y) / 2 + EXTENTION)
        left = int((self.lt_x + self.lb_x) / 2 - EXTENTION)
        right = int((self.rt_x + self.rb_x) / 2 + EXTENTION)
        lbox_bimg = self.before_img[top: bottom, left: right, :]
        lbox_aimg = self.after_img[top: bottom, left: right, :]
        pos_x, pos_y, width, height = self.get_food_info(lbox_bimg, lbox_aimg)
        if pos_x:
            direction = self.get_available_angle(self.empty_img[top: bottom, left: right, :], lbox_bimg, (pos_x, pos_y), (width, height))
            pos_x += left
            pos_y += top
        else:
            direction = ""
            width = 0
            height = 0
            pos_x = 0
            pos_y = 0
        cv2.drawMarker(self.output_img, (int(pos_x), int(pos_y)), (0, 0, 255), markerType=cv2.MARKER_STAR, markerSize=20)
        cv2.imwrite("/home/tanemoto/Desktop/images/output_" + str(self.count) + ".png", self.output_img)
        # publish
        if pos_x:
            pos_x -= (left + right) / 2
            pos_y -= (top + bottom) / 2
        self.pub_msg = FoodPacking()
        self.pub_msg.x = pos_x
        self.pub_msg.y = pos_y
        self.pub_msg.width = width
        self.pub_msg.height = height
        self.pub_msg.direction = direction

if __name__ == '__main__':
    rospy.init_node("get_placed_pos")
    vis = VisualFeedback()
    rospy.spin()
