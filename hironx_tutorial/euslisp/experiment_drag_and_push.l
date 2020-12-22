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

(setq *G-z* -47.2512)
(setq right 74.8102)
(setq back 460.225)
(setq *left* 169.872)
(setq *front* 340.46)
(setq *length* 119.765)
(setq *width* 95.0622)

(format t "G-z ~A~%" *G-z*)
(format t "right ~A back ~A~%" right back)
(format t "left ~A front ~A~%" *left* *front*)
(format t "size GA width ~A length ~A~%" *length* *width*)

(setq *catch-z* 0)

(load "get_foods_info.l")
(get-lbox-coords)

;; ----ここに情報を手入力する-----
;;(setq name_list (list "rolled_egg" "fried_chicken" "rolled_egg" "broccoli" "broccoli" "fried_chicken" "fried_chicken" "tomato" "tomato"))
;;(setq box_list (list (list 65.6859 27.4425 28.3371) (list 29.7736 38.8611 26.9225) (list 64.3962 32.2016 25.6383) (list 45.6781 49.6819 36.1149) (list 46.0856 43.0608 31.9909) (list 42.0373 27.6604 27.0514) (list 36.5796 37.7505 24.1324) (list 30.1875 27.3015 25.9203) (list 29.1224 31.8318 11.7092)))
(setq name_list (list "fried_chicken" "tomato" "rolled_egg" "tomato" "fried_chicken" "broccoli" "fried_chicken" "broccoli" "tomato" "rolled_egg"))
(setq box_list (list (list 44.8339 46.1884 24.5818) (list 31.0127 32.3634 32.05) (list 63.4509 30.4951 26.3596) (list 32.189 29.301 29.4766) (list 40.7252 47.4572 26.9578) (list 46.5322 42.2495 33.5564) (list 46.7047 39.042 25.6326) (list 42.4879 47.1751 27.5931) (list 29.2289 30.5835 26.2575) (list 64.5653 33.6759 24.8617)))
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
    (if (not (= 3D_x_size 0))
	(pull_over target_name (elt place_list num) lbox_pos diff_2D_x diff_2D_y 3D_y_size 3D_x_size (elt food-size 2) approach)
      (setq fail_catch_flag t))
    (place-init-pose)
    )
  )
  
