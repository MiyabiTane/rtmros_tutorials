#!/usr/bin/env roseus

;; You need to run stuff_by_GA.py
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(hironxjsk-init)
(objects (list *hironxjsk*))

;;init
(ros::roseus "hiro_lunchbox")

;;use tf
(unless (boundp '*tfl*)
  (setq *tfl* (instance ros::transform-listener :init)))

;;def some poses
(setq stop-move-flag nil)
(defun send-robot ()
  (if (< (elt (send (send (send *hironxjsk* :rarm :end-coords) :copy-worldcoords) :pos) 2) -10.0)
      (progn
	(format t "DANGER!!~%")
	(setq stop-move-flag t))
    (progn
      (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 2000)
      (send *ri* :wait-interpolation)))
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
(init-pose-20201030)

;;get lunchbox position and size
(load "recognize_lunchbox.l")
;;(recognize_lunchbox_main)
(setq *G-z* -40.5341)
(setq right -1.79236)
(setq back 504.795)
(setq *left* 71.7166)
(setq *front* 400.406)
(setq *length* 125.673)
(setq *width* 70.3547)

(format t "G-z ~A~%" *G-z*)
(format t "right ~A back ~A~%" right back)
(format t "left ~A front ~A~%" *left* *front*)
(format t "size GA width ~A length ~A~%" *length* *width*)

;;get catch-z coords
(load "check_table_height.l")
;;(check_table_height_main)
(setq *catch-z* -15.5801)
(format t "catch-z ~A~%" *catch-z*)

;;get food position and size
(load "get_foods_info.l")
(get_foods_info_main)
(format t "name_list ~A~%" name_list)
(format t "pos_list ~A~%" pos_list)
(format t "box_list ~A~%" box_list)

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
