#!/usr/bin/env roseus

;; You need to run stuff_by_GA.py
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(hironxjsk-init)
(objects (list *hironxjsk*))

;;init
(ros::roseus "hiro_lunchbox")

;;;;def some poses
(defun send-robot ()
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

(defun send-right-hand (sleep-time)
  (send *ri* :hand-angle-vector :rhand (send *hironxjsk* :hand-angle-vector :rhand) 2000)
  (ros::duration-sleep sleep-time)
  )

(defun send-left-hand (sleep-time)
  (send *ri* :hand-angle-vector :lhand (send *hironxjsk* :hand-angle-vector :lhand) 2000)
  (ros::duration-sleep sleep-time)
  )

(defun look-73b2-table ()
  ;;look table
  (send *hironxjsk* :head :look-at
        (send (send (send *hironxjsk* :torso :end-coords :copy-worldcoords)
                    :translate #f(750 0 0)) :worldpos))
  (send *hironxjsk* :head :look-at #f(600 0 0))
  ;;(send-robot)
  )

(defun init-pose-20201001 ()
  (send *hironxjsk* :reset-manip-pose)
  (send *hironxjsk* :larm_joint4 :joint-angle -35)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 100) :world)
  (send *hironxjsk* :rarm_joint1 :joint-angle 20) 
  (send *hironxjsk* :rarm :move-end-pos #f(150 0 300))
  (look-73b2-table)
  (send-robot)
  )

;;move hand out of camera view
(defun move_hand_out_of_camera ()
  (init-pose-20201001)
  (send *hironxjsk* :larm :move-end-pos #f(0 200 0))
  (send-robot)
  (send *hironxjsk* :rarm :move-end-pos #f(0 -200 0))
  (send-robot)
  (send *hironxjsk* :rarm_joint5 :joint-angle 0)
  (look-73b2-table)
  (send-robot)
  )

(defun init-pose-20201030 ()
  (send *hironxjsk* :reset-manip-pose)
  (send *hironxjsk* :larm_joint4 :joint-angle -35)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 100) :world)
  (send *hironxjsk* :rarm :move-end-pos #f(50 0 150) :world)
  (send *hironxjsk* :rarm_joint5 :joint-angle 100)
  (send *hironxjsk* :rarm_joint3 :joint-angle 35)
  (look-73b2-table)
  (send-robot)
  )

(defun open-hand ()
  ;;open hand
  (send *hironxjsk* :hand-angle-vector :lhand #f(20.449 -20.449 -20.449 20.449))
  (send-left-hand 2.0)
  (send *hironxjsk* :hand-angle-vector :rhand #f(20.449 -20.449 -20.449 20.449))
  (send-right-hand 2.0)
  )

;;To Do load all file at first

(look-73b2-table)

;;get food position and size
(load "get_foods_info.l")
(get_foods_info_main)
(format t "name_list ~A~%" name_list)
(format t "pos_list ~A~%" pos_list)
(format t "box_list ~A~%" box_list)

;;get lunchbox position and size
(load "recognize_lunchbox.l")
(recognize_lunchbox_main)
(format t "left ~A front ~A~%" *left* *front*)
(format t "size GA width ~A length ~A~%" *length* *width*)

;;get catch-z coords
(load "check_table_height.l")
(check_table_height_main)
(format t "catch-z ~A~%" *catch-z*)

;;publish food info and lunchbox info
(load "info_publisher.l")
(info_publisher_main)

;;Calculate stuff posiion using stuff_by_GA.py

;;subscribe result of GA caluculation
(load "result_subscriber.l")
(result_subscriber_main)
(format t "place_list is ~A~%" place_list)

;;catch and place foods
(load "catch_and_place.l")
(catch_and_place_main)
