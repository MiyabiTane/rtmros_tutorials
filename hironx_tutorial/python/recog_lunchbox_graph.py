#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import WrenchStamped

import numpy as np
import matplotlib.pyplot as plt

import time

class GetInfo:
    
    def __init__(self):
        self.x_lst = []
        self.y_lst = []
        self.z_lst = []
        self.norm_lst = []
        self.time_lst = []
        self.time_start = None
        self.time_now = None
        self.start_time = None
        self.time_before = None
        self.count = 0
        self.before_x = None
        self.before_y = None
        self.before_z = None
        self.before_norm = None

    def wrench_cb(self, msg):
        # Force = msg.wrench.force
        Torque = msg.wrench.torque
        self.time_now = time.time()
        if not self.time_before:
            #print("record")
            self.before_x = Torque.x
            self.before_y = Torque.y
            self.before_z = Torque.z
            self.before_norm = np.linalg.norm(np.array([Torque.x, Torque.y, Torque.z]))
            self.time_before = time.time()
            self.count += 1
            self.x_lst.append(0)
            self.y_lst.append(0)
            self.z_lst.append(0)
            self.norm_lst.append(0)
            self.time_lst.append(self.time_before)

        elif self.time_now - self.time_before >= 1.5 and self.count != 75:
            #print("record")
            self.x_lst.append(Torque.x - self.before_z)
            self.y_lst.append(Torque.y - self.before_y)
            self.z_lst.append(Torque.z - self.before_z)
            self.norm_lst.append(np.linalg.norm(np.array([Torque.x, Torque.y, Torque.z])) - self.before_norm)
            self.time_lst.append(self.time_now)
            self.count += 1
            #print(self.count)
            print(self.time_now - self.time_lst[0])
            self.time_before = time.time()
            self.before_x = Torque.x
            self.before_y = Torque.y
            self.before_z = Torque.z
            self.before_norm = np.linalg.norm(np.array([Torque.x, Torque.y, Torque.z]))
                 
    def main(self):
        rospy.init_node("HIRO")
        while True:
            rospy.Subscriber("/lhsensor",WrenchStamped, self.wrench_cb)
            rospy.sleep(0.1)
            if self.count == 75:
                break
        ziku = np.array(self.time_lst) - np.array([self.time_lst[0]]*len(self.time_lst))
        #print(ziku)
        #print(self.x_lst)
        leng = 75
        print(ziku.shape)
        print(len(self.x_lst))
        #plt.plot(ziku, self.x_lst, label="x")
        plt.plot(ziku[:leng], self.y_lst[:leng], label="y")
        #plt.plot(ziku, self.z_lst, label="z")
        plt.plot(ziku[:leng], self.norm_lst[:leng], label="norm")
        plt.xlabel("time[s]")
        plt.ylabel("torque")
        plt.legend()
        #plt.title("torque of left hand")
        plt.savefig("/home/tork/Desktop/recog_lunchbox_0117.png")

info = GetInfo()
info.main()

        
            
    
    
