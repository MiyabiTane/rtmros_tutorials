#!/usr/bin/env roseus

;; You need to run stuff_by_GA.py
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(hironxjsk-init)
(objects (list *hironxjsk*))

;;init
(ros::roseus "hiro_lunchbox")

;;get catch-z coords
(load "check_table_height.l")
(format t "catch-z ~A~%" *catch-z*)

;;get food position and size
(load "get_foods_info.l")
(format t "name_list ~A~%" name_list)
(format t "pos_list ~A~%" pos_list)
(format t "box_list ~A~%" box_list)

;;get lunchbox position and size
(load "recognize_lunchbox.l")
(format t "left ~A front ~A~%" *left* *front*)
(format t "size GA width ~A length ~A~%" *length* *width*)

;;publish food info and lunchbox info
(load "info_publisher.l")

;;Calculate stuff posiion using stuff_by_GA.py

;;subscribe result of GA caluculation
(load "result_subscriber.l")
(format t "place_list is ~A~%" place_list)

;;catch and place foods
(load "catch_and_place.l")
