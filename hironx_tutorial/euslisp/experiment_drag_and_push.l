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
  (setq right-z (elt (send (send (send *hironxjsk* :rarm :end-coords) :copy-worldcoords) :pos) 2))
  (setq left-z (elt (send (send (send *hironxjsk* :larm :end-coords) :copy-worldcoords) :pos) 2))
  (if (and (> right-z -10) (> left-z 15))
      (progn
	(send *ri* :angle-vector (send *hironxjsk* :angle-vector) 2000)
	(send *ri* :wait-interpolation))
    (progn
      (format t "DANGER!!~%")
      (setq stop-move-flag t)))
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
  (send *hironxjsk* :head :look-at #f(500 0 0))
  ;;(send-robot)
  )

(defun init-pose-20201030 ()
  (send *hironxjsk* :reset-manip-pose)
  (send *hironxjsk* :larm_joint4 :joint-angle -35)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 100) :world)
  (send *hironxjsk* :rarm :move-end-pos #f(50 0 150) :world)
  (send *hironxjsk* :rarm_joint5 :joint-angle -75)
  (send *hironxjsk* :rarm_joint3 :joint-angle 35)
  (send *hironxjsk* :rarm_joint4 :joint-angle 48)
  (look-73b2-table)
  (send-robot)
  )

;;move hand out of camera view
(defun move_hand_out_of_camera ()
  (send *hironxjsk* :reset-manip-pose)
  (look-73b2-table)
  (send *hironxjsk* :larm_joint4 :joint-angle -35)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 100) :world)
  (send *hironxjsk* :rarm :move-end-pos #f(50 0 150) :world)
  (send *hironxjsk* :rarm_joint5 :joint-angle -75)
  (send *hironxjsk* :rarm_joint3 :joint-angle 35)
  (send *hironxjsk* :rarm_joint4 :joint-angle 48)
  (send *hironxjsk* :rarm :move-end-pos #f(0 -160 0) :world)
  (send *hironxjsk* :larm :move-end-pos #f(0 200 0))
  (send *hironxjsk* :rarm_joint5 :joint-angle -163)
  (send-robot)
  )

(defun open-hand ()
  ;;open hand
  (send *hironxjsk* :hand-angle-vector :lhand #f(10.449 -10.449 -10.449 10.449))
  (send-left-hand 2.0)
  (send *hironxjsk* :hand-angle-vector :rhand #f(10.449 -10.449 -10.449 10.449))
  (send-right-hand 2.0)
  )

(defun get-lbox-coords ()
  (sub-lbox-pos back *left* *G-z*)
  (setq ltop (float-vector lbox_x lbox_y))
  (sub-lbox-pos *front* *left* *G-z*)
  (setq lbottom (float-vector lbox_x lbox_y))
  (sub-lbox-pos back right *G-z*)
  (setq rtop (float-vector lbox_x lbox_y))
  (sub-lbox-pos *front* right *G-z*)
  (setq rbottom (float-vector lbox_x lbox_y))
  (setq lbox_length (norm (v- ltop lbottom)))
  (setq lbox_height (norm (v- ltop rtop)))
  (format t "ltop lbottom rtop rbottom ~A ~A ~A ~A~%" ltop lbottom rtop rbottom)
  (setq lbox-center (scale (/ 1.0 4) (v+ (v+ (v+ ltop lbottom) rtop) rbottom)))
  )

(init-pose-20201030)
(open-hand)

(load "recognize_lunchbox.l")
;;(recognize_lunchbox_main)

(setq *G-z* -52.3231)
(setq right 80.5908)
(setq back 457.384)
(setq *left* 180.657)
(setq *front* 342.599)
(setq *length* 114.785)
(setq *width* 100.066)

(format t "G-z ~A~%" *G-z*)
(format t "right ~A back ~A~%" right back)
(format t "left ~A front ~A~%" *left* *front*)
(format t "size GA width ~A length ~A~%" *length* *width*)

(setq *catch-z* 0)

(load "get_foods_info.l")
(get-lbox-coords)

;; ----ここに情報を手入力する-----
;;(setq name_list (list "rolled_egg" "rolled_egg" "fried_chicken" "tomato" "broccoli" "broccoli" "tomato" "fried_chicken" "tomato" "tomato")) ;;"fried_chicken" "tomato" "tomato"))
;;(setq box_list (list (list 67.3376 34.1309 27.2797) (list 67.1005 33.2087 25.3866) (list 41.7253 42.171 26.0795) (list 32.3176 29.6084 25.6023) (list 43.0249 37.014 31.8576) (list 42.2204 47.1033 26.6139) (list 28.531 35.8058 28.0338) (list 43.7579 41.9366 26.4388) (list 27.191 34.4848 28.5558) (list 29.4047 33.6799 2.47896))) ;;(list 38.5252 46.6802 37.1121) (list 27.191 34.4848 28.5558) (list 29.4047 33.6799 2.47896)))
(setq name_list (list "tomato" "tomato" "broccoli" "broccoli" "fried_chicken" "fried_chicken" "rolled_egg" "rolled_egg" "octopus_wiener" "octopus_wiener"))
(setq box_list (list (list 27.0145 36.123 30.4755) (list 28.0106 33.8311 33.2695) (list 45.0872 37.681 24.1513) (list 33.8893 36.7995 23.8663) (list 35.5316 41.5575 26.7719) (list 36.559 44.3106 30.3144) (list 64.1084 29.1797 25.4524) (list 67.8967 37.9132 26.0683) (list 22.4268 22.4268 3.05885) (list 20.9132 25.2793 30.3283)))
;; ------------------------------

;;publish food info and lunchbox info
(load "info_publisher.l")
(info_publisher_main)

;;Calculate stuff posiion using stuff_by_GA.py

;;subscribe result of GA caluculation
(load "result_subscriber.l")
(result_subscriber_main)

(load "catch_and_place.l")
(setq *tong-offset* #f(-20.23 -114.702 90.2251))
(setq *chopstick-offset* #f(-12.624 18.437 84.2388))
(setq *chopstick-offset-r* #f(-25.065 34.9855 84.2388))
(format t "- - - CATCH AND PLACE - - -~%")
(place-init-pose)
(pub-lbox-coords)
(sub-img 2)
(dotimes (i (length order_list))
  (setq num (elt order_list i))
  (format t "num : ~A~%" num)
  (when (> (elt (elt place_list num) 0) 0)
    (setq target_name (elt name_list num))
    (setq food-size (elt box_list num))
    (format t "target : ~A~%" target_name)
    (format t "size : ~A place : ~A~%" food-size (elt place_list num))
    (sub-img 0)
    (format t "PLEASE PUT FOOD in 10 seconds~%")
    (ros::duration-sleep 10.0)
    (sub-img 1)
    (send (send pub_status_msg :header) :stamp (ros::time-now))
    (ros::subscribe "/get_placed_pos/output" hironx_tutorial::FoodPacking #'place_cb)
    (setq catch_flag nil)
    (until catch_flag
	   (ros::publish "/get_placed_pos/input" pub_status_msg)
	   (ros::spin-once)
	   (ros::sleep))
    (ros::unsubscribe "/get_placed_pos/output")
    (setq lbox_pos (float-vector (/ (+ *front* back) 2) (/ (+ *left* right) 2) 0))
    (if (and (not (= 2D_width 0)) (not (string= target_name "tomato")))
	(pull_over target_name (elt place_list num) lbox_pos diff_2D_x diff_2D_y 2D_width 2D_height (elt food-size 2) approach)
      (setq fail_catch_flag t))
    (place-init-pose)
    )
  )
  
