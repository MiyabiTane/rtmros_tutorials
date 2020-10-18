#!/usr/bin/env roseus


(ros::roseus-add-msgs "jsk_recognition_msgs")
;;(ros::roseus "hiro_lunchbox")
(ros::advertise "/lunchbox_info" jsk_recognition_msgs::Rect 1)
(ros::advertise "/food_size_info" geometry_msgs::PoseArray 1)
(ros::advertise "/food_name_info" jsk_recognition_msgs::LabelArray 1)
(ros::rate 10)
;;subscribe flag
(setq *sub-flag-topic* "/calc_finish_flag")
(ros::roseus-add-msgs "std_msgs")

(setq pub_box_msg (instance jsk_recognition_msgs::Rect :init))
(send pub_box_msg :x *front*)
(send pub_box_msg :y *left*)
(send pub_box_msg :width *length*)
(send pub_box_msg :height *width*)

(setq pub_food_msg (instance geometry_msgs::PoseArray :init))
(dotimes (i (length box_list))
  (setq food_msg (instance geometry_msgs::Pose :init))
  (send (send food_msg :position) :x (elt (elt box_list i) 0))
  (send (send food_msg :position) :y (elt (elt box_list i) 1))
  (send (send food_msg :position) :z (elt (elt box_list i) 2))
  ;;(format t "food_msg is ~A~%" food_msg)
  (send pub_food_msg :poses (append (send pub_food_msg :poses) (list food_msg)))
  )

(setq pub_name_msg (instance jsk_recognition_msgs::LabelArray :init))
(dotimes (i (length name_list))
  (setq name_msg (instance jsk_recognition_msgs::Label :init))
  (send name_msg :name (elt name_list i))
  (send pub_name_msg :labels (append (send pub_name_msg :labels) (list name_msg)))
  )

(defun flag_cb(msg)
  (format t "Called")
  (if (eql (send msg :data) 1)
      (ros::exit))
  )

(while (ros::ok)
  (ros::publish "/lunchbox_info" pub_box_msg)
  (ros::publish "/food_size_info" pub_food_msg)
  (ros::publish "/food_name_info" pub_name_msg)
  (ros::subscribe *sub-flag-topic* std_msgs::Int16 #'flag_cb)
  (ros::spin-once)
  (ros::sleep)
  )

;;input examples
;;(setq box_list (list (list 4 2 2) (list 4 4 4) (list 4 4 4) (list 3 3 3) (list 2 2 3.5) (list 4 4 4) (list 4 2 2) (list 4 4 4) (list 2 2 3.5) (list 3 3 3)))
;;(setq name_list (list "rolled_egg" "fried_chicken" "broccoli" "tomato" "octopus_wiener" "fried_chicken" "rolled_egg" "broccoli" "octopus_wiener" "tomato"))
