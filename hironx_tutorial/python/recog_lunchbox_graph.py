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
        self._x_lst = []
        self._y_lst = []
        self._z_lst = []
        self.time_lst = []

        self.time_before = None
        self.time_now = None
        self.count = 0

    def wrench_cb(self, msg):
        Force = msg.wrench.force
        Torque = msg.wrench.torque
        self.time_now = time.time()
        if not self.time_before:
            #print("record")
            self.x_lst.append(Force.x)
            self.y_lst.append(Force.y)
            self.z_lst.append(Force.z)
            self._x_lst.append(Torque.x)
            self._y_lst.append(Torque.y)
            self._z_lst.append(Torque.z)
            #self.norm_lst.append(np.linalg.norm(np.array([Force.x, Force.y, Force.z])))
            self.time_lst.append(self.time_now)
            self.count += 1
            self.time_before = time.time()
        else:
            if self.time_now - self.time_before >= 0.5:
                #print("record")
                self.x_lst.append(Force.x)
                self.y_lst.append(Force.y)
                self.z_lst.append(Force.z)
                self._x_lst.append(Torque.x)
                self._y_lst.append(Torque.y)
                self._z_lst.append(Torque.z)
                #self.norm_lst.append(np.linalg.norm(np.array([Force.x, Force.y, Force.z])))
                self.time_lst.append(self.time_now)
                self.count += 1
                self.time_before = time.time()
                 
    def main(self):
        rospy.init_node("HIRO")
        count = 0
        while True:
            rospy.Subscriber("/lhsensor",WrenchStamped, self.wrench_cb)
            rospy.sleep(0.1)
            if self.count == 340:
                break
        ziku = np.array(self.time_lst) - np.array([self.time_lst[0]]*len(self.time_lst))
        print(ziku)
        print(self.x_lst)
        leng = len(ziku)
        self.x_lst = self.x_lst[:leng]
        self.y_lst = self.y_lst[:leng]
        self.z_lst = self.z_lst[:leng]
        self._x_lst = self._x_lst[:leng]
        self._y_lst = self._y_lst[:leng]
        self._z_lst = self._z_lst[:leng]
        #self.norm_lst = self.norm_lst[:leng]
        plt.subplot(1,2,1)
        plt.plot(ziku, self.x_lst, label="x")
        plt.plot(ziku, self.y_lst, label="y")
        plt.plot(ziku, self.z_lst, label="z")
        #plt.plot(ziku, self.norm_lst, label="norm")
        plt.xlabel("time[s]")
        plt.ylabel("force")
        plt.legend()
        plt.title("force of left hand")

        plt.subplot(1,2,2)
        plt.plot(ziku, self._x_lst[:340], label="x")
        plt.plot(ziku, self._y_lst[:340], label="y")
        plt.plot(ziku, self._z_lst[:340], label="z")
        plt.xlabel("time[s]")
        plt.ylabel("torque")
        plt.legend()
        plt.title("torque of left hand")
        plt.savefig("/home/tanemoto/Desktop/graphs/recog_lunchbox_1017.png")

info = GetInfo()
info.main()

        
            
    
    
