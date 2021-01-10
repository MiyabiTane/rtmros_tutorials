#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import random
from copy import deepcopy
import numpy as np

from BL_algorithm import BL_main
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Header

from jsk_recognition_msgs.msg import Rect
from jsk_recognition_msgs.msg import LabelArray
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int16MultiArray

import hiro_talk
import cv2

SAVE_NAME = "/home/tanemoto/Desktop/images/place_output.png"
ORDER_SAVE_NAME = "/home/tanemoto/Desktop/images/order_output.png"
ADD_SIZE = 5

"""
x: width, y: height
"""
class StuffFood():
    """
    sample input:
        name_list = ["rolled_egg", "fried_chicken", "broccoli", "tomato", "octopus_wiener", "fried_chicken", "rolled_egg", "broccoli", "octopus_wiener", "tomato"]
        box_list = [[4,2,2], [4,4,4], [4,4,4], [3,3,3], [2,2,3.5], [4,4,4], [4,2,2], [4,4,4], [2,2,3.5], [3,3,3]]
        box_size = [12,11]
        like_list = [["octopus_wiener", "octopus_wiener"], ["rolled_egg", "rolled_egg"], ["tomato", "tomato"]]
        dislike_list = [["octopus_wiener", "fried_chicken"]]
        want_to_eat = [["fried_chicken", "tomato"]]
    sample output:
        best_stuff = [(6.0, 1.0), (2.0, 6.0), (2.0, 2.0), (1.5, 9.5), (9.0, 3.0), (8.0, 10.0), (6.0, 3.0), (6.0, 6.0), (9.0, 1.0), (4.5, 9.5)]
    """
    def __init__(self, name_list, box_list, box_size, like_list, dislike_list, want_to_eat, indivisuals, generation, elite):
        self.indivisuals = indivisuals
        self.generation = generation
        self.elite = elite
        self.box_size = [box_size[0] + ADD_SIZE, box_size[1] + ADD_SIZE]
        self.want_to_eat = want_to_eat
        self.name_to_index_dict = {}
        for i, food in enumerate(name_list):
            if food in self.name_to_index_dict:
                self.name_to_index_dict[food] += [i]
            else:
                self.name_to_index_dict[food] = [i]
        self.box_dict = {}
        for i in range(len(name_list)):
            self.box_dict[i] = box_list[i][:2]
        #print(self.box_dict)
        self.cand_list = []
        init_list = [i for i in range(len(name_list))]
        for i in range(indivisuals):
            copy = deepcopy(init_list)
            random.shuffle(copy)
            self.cand_list.append(copy)
        self.cand_list = np.array(self.cand_list)
        self.best_keeper = (None, -float('inf'), None) #(best cand_list, eval_point, points)
        #print(self.cand_list)
        self.like_list = like_list
        self.dislike_list = dislike_list
        #for visalize
        self.name_list = name_list


    def evaluate(self):
    #output : list [int, int, int ...] sum(int) = 1
    #         represents each indivisual's probability of being next parent
        def calc_point(list, stuff_pos, plus_flag):
            point = 0
            for food1, food2 in list:
                for index_1 in self.name_to_index_dict[food1]:
                    for index_2 in self.name_to_index_dict[food2]:
                        if not index_1 == index_2: 
                            if stuff_pos[index_1][0] + self.box_dict[index_1][0] == stuff_pos[index_2][0] or stuff_pos[index_2][0] + self.box_dict[index_2][0] == stuff_pos[index_1][0]:
                                # | food1 | food2 |
                                if min(stuff_pos[index_1][1] + self.box_dict[index_1][1], stuff_pos[index_2][1] + self.box_dict[index_2][1]) - max(stuff_pos[index_1][1], stuff_pos[index_2][1]) >= min(self.box_dict[index_1][1], self.box_dict[index_2][1]) / 2:
                                    if food1 == food2: #avoid double count
                                        point = point + 0.5 if plus_flag else point - 0.5
                                    else:
                                        point = point + 1 if plus_flag else point - 1
                            elif stuff_pos[index_1][1] + self.box_dict[index_1][1] == stuff_pos[index_2][1] or stuff_pos[index_2][1] + self.box_dict[index_2][1] == stuff_pos[index_1][1]:
                                # food1
                                # -----
                                # food2
                                if min(stuff_pos[index_1][0] + self.box_dict[index_1][0], stuff_pos[index_2][0] + self.box_dict[index_2][0]) - max(stuff_pos[index_1][0], stuff_pos[index_2][0]) >= min(self.box_dict[index_1][0], self.box_dict[index_2][0]) / 2:
                                    if food1 == food2:
                                        point = point + 0.5 if plus_flag else point - 0.5
                                    else:
                                        point = point + 1 if plus_flag else point - 1
            return point

        points = []
        for lst in self.cand_list:
            point = 0
            stuff_pos = BL_main(lst, self.box_dict, self.box_size)
            #if satisfy the like condition, point +=1
            point += calc_point(self.like_list, stuff_pos, plus_flag = True)
            #if satisfy the dislike condition, point -=1
            point += calc_point(self.dislike_list, stuff_pos, plus_flag = False)
            #if favorite food is not in lunchbox point -= 1
            capacity = self.box_size[0] * self.box_size[1]
            for i in range(len(stuff_pos)):
                if stuff_pos[i][0] + self.box_dict[i][0] > self.box_size[0] or stuff_pos[i][1] + self.box_dict[i][1] > self.box_size[1]:
                    if self.name_list[i] in self.want_to_eat:
                        point -= 2
                else:
                    capacity -= self.box_dict[i][0] * self.box_dict[i][1]
            area_percent = float(capacity) / (self.box_size[0] * self.box_size[1])
            # print(area_percent)
            point -= area_percent * 30
            points.append(point)
        #for_choose_parameter
        #print(points)
        max_point = max(points)
        points = map(lambda x: x-(min(points)), points)
        if float(sum(points)) == 0:
            points = [1.0/len(points)] * len(points)
        else:
            points = map(lambda x: float(x)/float(sum(points)), points)
        #print(points)
        return points, max_point


    def partial_crossover(self, parent_1, parent_2):
        #input  : list of parents
        #output : list of children 
        num = len(parent_1)
        cross_point = random.randrange(1, num-1)
        child_1 = parent_1
        child_2 = parent_2
        for i in range(num - cross_point):
            target_index = cross_point + i
            value_1 = parent_1[target_index]
            value_2 = parent_2[target_index]
            index_1 = np.where(parent_1 == value_2)
            index_2 = np.where(parent_2 == value_1)
            child_1[target_index] = value_2
            child_2[target_index] = value_1
            child_1[index_1] = value_1
            child_2[index_2] = value_2
        return child_1, child_2


    def generate_next_generation(self):
    #update self.cand_list    
        points, max_point = self.evaluate()
        # print(max_point)
        copy = deepcopy(self.cand_list)
        for i in range((self.indivisuals - self.elite)//2):
            index_1, index_2 = np.random.choice(len(points), 2, replace = True, p = points)
            #print(index_1, index_2)
            parent_1 = self.cand_list[index_1]
            parent_2 = self.cand_list[index_2]
            child_1, child_2 = self.partial_crossover(parent_1, parent_2)
            copy[2*i] = child_1
            copy[2*i + 1] = child_2
        # inherit high point parents
        elite_parent_index = np.argsort(points)
        for i in range(self.indivisuals - self.elite, self.indivisuals):
            copy[i] = self.cand_list[elite_parent_index[i]]
        self.cand_list = deepcopy(copy)
        _, cur_point, _ = self.best_keeper
        if max_point >= cur_point:
            self.best_keeper = (self.cand_list, max_point, points)
        #print(self.cand_list)


    def mutation(self, num_mutation = 3, mute_per = 0.7):
    # num_mutaiton can mutate at the percentage of mute_per 
        mutation_genes = np.random.choice(len(self.cand_list), num_mutation, replace = False)
        for index in mutation_genes:
            copy = deepcopy(self.cand_list[index])
            flag = np.random.choice(2, 1, p = [1 - mute_per, mute_per])
            if flag == 1:
                #print("mutate ", index)
                values = np.random.choice(copy, 2, replace = False)
                copy[np.where(self.cand_list[index] == values[0])] = values[1]
                copy[np.where(self.cand_list[index] == values[1])] = values[0]
            self.cand_list[index] = deepcopy(copy)


    def GA_calc(self):
        for _ in range(self.generation):
            self.generate_next_generation()
            self.mutation()
        #select the one whose point is highest
        ans, point, points = self.best_keeper 
        print(ans, "point : ", point)
        print(self.cand_list[(np.argmax(points))])
        stuff = BL_main(self.cand_list[(np.argmax(points))], self.box_dict, self.box_size)
        # デバッグ
        best_stuff = deepcopy(stuff)
        cannot_stuff = []
        for i in range(len(best_stuff)):
            if best_stuff[i][0] + self.box_dict[i][0] > self.box_size[0] or best_stuff[i][1] + self.box_dict[i][1] > self.box_size[1]:
                cannot_stuff += [i]
                best_stuff[i] = (0,0)
            else:
                best_stuff[i] = (best_stuff[i][0] + self.box_dict[i][0]/2, best_stuff[i][1] + self.box_dict[i][1]/2)
        # print("stuff : {}".format(best_stuff))
        visualize(self.box_size, self.box_dict.values(), best_stuff, cannot_stuff, self.name_list, SAVE_NAME)
        return stuff, point, self.box_dict
        

# improvement
def rotate_food(box_list, count):
    new_box_list = deepcopy(box_list)
    changed_flag = False
    change_num = len(box_list) / 4
    random_int = [random.randint(0, len(box_list) - 1) for i in range(change_num)]
    for num in random_int:
        width = box_list[num][0]
        height = box_list[num][1]
        if float(width) / height < 1.5:
            new_box_list[num] = [height, width, box_list[num][2]]
            changed_flag = True
    if (not changed_flag) and count < 7:
        rotate_food(box_list, count + 1)
    print("changed rotation")
    return new_box_list


def visualize(box_size, box_list, best_stuff, cannot_stuff, name_list, name):
    down = 300
    img = np.full((300,300, 3), 200, dtype=np.uint8)
    cv2.rectangle(img, (0, 0), (box_size[0], box_size[1]), (0, 0, 0))
    color_dict = {"tomato": (0,0,255), "rolled_egg": (0,255,255), "octopus_wiener": (255,0,255), "fried_chicken": (0,0,128), "broccoli": (0,128,0)}
    for i in range(len(best_stuff)):
        if i in cannot_stuff:
            point_1 = (300 - int(box_list[i][0]), int(down))
            point_2 = (300, int(down) - int(box_list[i][1]))
            down -= box_list[i][1]
        else:
            point_1 = (int(best_stuff[i][0] - box_list[i][0]//2), int(best_stuff[i][1] - box_list[i][1]//2))
            point_2 = (int(best_stuff[i][0] + box_list[i][0]//2), int(best_stuff[i][1] + box_list[i][1]//2))
        color = color_dict[name_list[i]]
        cv2.rectangle(img, point_1, point_2, color, thickness=-1)
        cv2.rectangle(img, point_1, point_2, (255,255,0))
    cv2.imwrite(name, img)


def calc_place_order(box_size, box_list, name_list, stuff_pos):
# Decide the boundaries of the column based on the average size of the side dish
    food_height = np.array(box_list)[:, 1]
    ave_height = np.mean(food_height) - 10
    num = int(box_size[1] // ave_height) + 1
    place_row_lst = [[]] * num
    slip_lst = []
    order_lst = np.array([])
    for i, pos in enumerate(stuff_pos):
        # The robot have known that tomato is slippery
        if name_list[i] == "tomato":
            slip_lst.append(i)
        elif pos[0] != 0:
            row = int(pos[1] // ave_height)
            lst = deepcopy(place_row_lst[row])
            lst.append((i, pos[0], pos[1]))
            place_row_lst[row] = lst
    # print("place_row_lst:", place_row_lst)
    if not place_row_lst[-1]:
        place_row_lst = place_row_lst[:-1]
    count = 0
    for _ in range(len(place_row_lst)):
        place_row = place_row_lst[count]
        # sort by distance 
        if place_row:
            place_row.sort(key=lambda x: abs(x[1] - (box_size[0] / 2)))
            index = np.array(place_row)[::-1][:, 0]
            # print("index :", index)
            order_lst = np.concatenate([order_lst, index])
        count = count * -1 if count < 0 else (count * -1) - 1
    order_lst = np.concatenate([order_lst, np.array(slip_lst)]).astype(np.int16)
    return order_lst


def visualize_place_order(box_list, open_name, save_name, best_stuff, order_lst):
    img = cv2.imread(open_name)
    for i, index in enumerate(order_lst):
        left_bottom = (int(best_stuff[index][0] - box_list[index][0]/2), int(best_stuff[index][1] + box_list[index][1]/2))
        cv2.putText(img, str(i), left_bottom, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), thickness=2)
    cv2.imwrite(save_name, img)


def GA_main(name_list, box_list, box_size, like_list, dislike_list, want_to_eat, indivisuals=20, generation=200, elite=6, change_direc=10):
    cur_box_list = box_list
    max_point = -10000
    best_stuff = None
    best_box_dict = None
    for _ in range(change_direc):
        stuff_food = StuffFood(name_list, cur_box_list, box_size, like_list, dislike_list, want_to_eat, indivisuals, generation, elite)
        stuff, point, box_dict = stuff_food.GA_calc()
        # print("STUFF: ", stuff)
        if point > max_point:
            max_point = point
            best_stuff = stuff
            best_box_dict = box_dict
        cur_box_list = rotate_food(cur_box_list, count=0)
    #if food overflow, keep the index in cannot_stuff
    b_size = [box_size[0] + ADD_SIZE, box_size[1] + ADD_SIZE]
    cannot_stuff = []
    for i in range(len(best_stuff)):
        if best_stuff[i][0] + best_box_dict[i][0] > b_size[0] or best_stuff[i][1] + best_box_dict[i][1] > b_size[1]:
            cannot_stuff += [i]
            best_stuff[i] = (0,0)
        else:
            best_stuff[i] = (best_stuff[i][0] + best_box_dict[i][0]/2, best_stuff[i][1] + best_box_dict[i][1]/2)
    print("- - - BEST STUFF - - -")
    print("{} : {}point : ".format(best_stuff, max_point))
    visualize(b_size, best_box_dict.values(), best_stuff, cannot_stuff, name_list, SAVE_NAME)
    # calc place order
    order_lst = calc_place_order(box_size, best_box_dict.values(), name_list, best_stuff)
    visualize_place_order(best_box_dict.values(), SAVE_NAME, ORDER_SAVE_NAME, best_stuff, order_lst)
    return best_stuff, order_lst


class SubscribeVisualInfo():

    def __init__(self):
        self.box_list = None
        self.name_list = None
        self.box_size = None
        self.flag1 = True
        self.flag2 = True
        self.flag3 = True

    def lunchbox_info_cb(self, msg):
        self.box_size = [msg.width, msg.height]

    def name_info_cb(self, msg):
        self.name_list = msg.labels
        copy = []
        for i in range(len(self.name_list)):
            copy.append(self.name_list[i].name)
        self.name_list = copy 

    def size_info_cb(self, msg):
        self.box_list = msg.poses
        copy = []
        for i in range(len(self.box_list)):
            lst = [self.box_list[i].position.x, self.box_list[i].position.y, self.box_list[i].position.z]
            copy.append(lst)
        self.box_list = copy    
            
    def get_vis_info(self):
        rospy.init_node("hiro_lunchbox")
        lbox_msg = rospy.wait_for_message("/lunchbox_info", Rect)
        name_msg = rospy.wait_for_message("/food_name_info", LabelArray)
        food_size_msg = rospy.wait_for_message("/food_size_info", PoseArray)
        self.lunchbox_info_cb(lbox_msg)
        self.name_info_cb(name_msg)
        self.size_info_cb(food_size_msg)
        print("box_size is ", self.box_size)
        print("names are ", self.name_list) 
        print("food size  ", self.box_list)
        if self.box_size and self.name_list and self.box_list:
            return self.box_size, self.name_list, self.box_list
        self.get_vis_info()


def get_talk_info(name_list):
    Talk = hiro_talk.TalkWith()
    like_list, dislike_list, want_to_eat = Talk.main_before_stuff(name_list)
    return like_list, dislike_list, want_to_eat


def main():
    # print("Called GA main")
    #subscribe info from Coral
    visual_info = SubscribeVisualInfo()
    box_size, name_list, box_list = visual_info.get_vis_info()
    #subscribe info from talking
    like_list, dislike_list, want_to_eat = get_talk_info(name_list)
    #like_list = [["tomato", "tomato"]]
    #dislike_list = [["tomato", "tomato"]]
    #want_to_eat = [["tomato"]]
    #calc stuff pos using GA and BL
    print("like_list : ", like_list)
    print("dislike_list : ", dislike_list)
    print("want_to_eat : ",want_to_eat)
    best_stuff, order_lst = GA_main(name_list, box_list, box_size, like_list, dislike_list, want_to_eat)
    #publish stuff canter coords and box width and height
    pub = rospy.Publisher('/stuff_food_pos', PoseArray, queue_size = 1)
    pose_msg = PoseArray()
    for i, stuff_pos in enumerate(best_stuff):
        pose = Pose()
        pose.position.x = stuff_pos[0]
        pose.position.y = stuff_pos[1]
        pose.position.z = box_list[i][2]
        pose_msg.poses.append(pose)
    # publish order
    pub2 = rospy.Publisher('/stuff_order', Int16MultiArray, queue_size = 1)
    order_msg = Int16MultiArray()
    order_msg.data = order_lst
    # publish flag
    pub3 = rospy.Publisher('/calc_finish_flag', Int16, queue_size = 1)
    flag_msg = Int16()
    flag_msg.data = 1
    # publish
    while not rospy.is_shutdown():
        pub.publish(pose_msg)
        pub2.publish(order_msg) 
        pub3.publish(flag_msg)
        rospy.sleep(0.1)

main()
