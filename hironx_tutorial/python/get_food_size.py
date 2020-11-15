#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from jsk_recognition_msgs.msg import RectArray
from cv_bridge import CvBridge

from copy import deepcopy
import cv2
import numpy as np
TH = 200

class ImageProcessing:
    def __init__(self):
        self.got_img_flag = False
        self.got_rects_flag = False
        self.cv_image = None
        self.output_img = None
        self.rects_info = None
        self.header = None
        self.two_length_list = None
        self.bridge = CvBridge()
        
    def image_cb(self, msg):
        print("Img Called")  
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8") 
       
    def coral_cb(self, msg):
        print("Coral Called")
        self.rects_info = msg.rects
        self.header = msg.header
        self.two_length_list = [[]] * len(self.rects_info)
        # draw coral result
        self.output_img = deepcopy(self.cv_image)
        for rect in self.rects_info:
            cv2.rectangle(self.output_img, (rect.x, rect.y), (rect.x + rect.width, rect.y + rect.height), (255,0,0))

    def subscinfo(self):
        img_msg = rospy.wait_for_message("/head_camera/rgb/image_raw", Image)
        # coral_msg = rospy.wait_for_message("/edgetpu_object_detector/output/rects", RectArray)
        coral_msg = rospy.wait_for_message("/coral_rects_info", RectArray)
        self.image_cb(img_msg)
        self.coral_cb(coral_msg)


    def get_foods_rects(self):
        if not self.rects_info:
            self.subscinfo()
        # self.cv_image = cv2.imread("/home/tork/Desktop/images/output_color.png")
        black_img = deepcopy(self.cv_image)
        # draw white plate black
        black_img[np.where(black_img[:,:,0] > TH) or np.where(black_img[:,:,1] > TH) or np.where(black_img[:,:,2] > TH)] = 0
        img_gray = cv2.cvtColor(black_img, cv2.COLOR_BGR2GRAY)
        # draw others black
        _thre, bw_img = cv2.threshold(img_gray, 1, 255, cv2.THRESH_BINARY)
        # get contours
        # if opencv version is latest, _, contours, _hierarchy -> contours, _hierarchy
        _, contours, _hierarchy = cv2.findContours(bw_img ,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        boxes_cand = []
        max_foods_size = self.rects_info[0].width * self.rects_info[0].height * 2
        for cnt in contours:
            #remove too small objects and too big object
            if 100 < cv2.contourArea(cnt) < max_foods_size:
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                boxes_cand.append(np.array(box))
        
        for i, rect in enumerate(self.rects_info):
            for box in boxes_cand:
                bcenter_x = np.mean(box[:, 0])
                bcenter_y = np.mean(box[:, 1])
                if rect.x <= bcenter_x <= rect.x + rect.width:
                    if rect.y <= bcenter_y <= rect.y + rect.height:
                        for point in box:
                            if point[0] < bcenter_x and point[1] < bcenter_y:
                                left_top = point
                            elif point[0] < bcenter_x and point[1] > bcenter_y:
                                left_bottom = point
                            elif point[0] > bcenter_x and point[1] > bcenter_y:
                                right_bottom = point
                        # visualize result
                        cv2.line(self.output_img, (left_top[0], left_top[1]), (left_bottom[0], left_bottom[1]), (0, 255, 0), thickness=2)
                        cv2.line(self.output_img, (left_bottom[0], left_bottom[1]), (right_bottom[0], right_bottom[1]), (0, 255, 0), thickness=2)
                        width = right_bottom[0] - left_bottom[0]
                        length = left_bottom[1] - left_top[1] 
                        self.two_length_list[i] = (width, length) 
        
    def publish_result(self):
        pub_msgs = RectArray()
        pub_msgs.header = self.header
        pub_msgs.rects = self.rects_info
        for info, rect_origin in zip(self.two_length_list, self.rects_info):
            pub_msg = rect_origin
            if info:
                pub_msg.width = info[0]
                pub_msg.height = info[1]
            pub_msgs.rects.append(pub_msg)
        # visualize result
        # cv2.imwrite("/home/tork/Desktop/images/output.png", self.output_img)
        pub = rospy.Publisher("/result_of_imageprocessing", RectArray, queue_size=1)
        while not rospy.is_shutdown():
            pub.publish(pub_msgs)
            rospy.sleep(0.1)

    
if __name__ == '__main__':
    rospy.init_node("get_food_rects")
    img_pro = ImageProcessing()
    img_pro.subscinfo()
    img_pro.get_foods_rects()
    img_pro.publish_result()



