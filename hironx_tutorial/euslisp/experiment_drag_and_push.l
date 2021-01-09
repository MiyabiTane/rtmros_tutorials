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

(setq *G-z* -45.5393)
(setq right 50.9496)
(setq back 480.185)
(setq *left* 145.999)
(setq *front* 375.327)
(setq *length* 104.858)
(setq *width* 95.0489)

(format t "G-z ~A~%" *G-z*)
(format t "right ~A back ~A~%" right back)
(format t "left ~A front ~A~%" *left* *front*)
(format t "size GA width ~A length ~A~%" *length* *width*)

(setq *catch-z* 0)

(load "get_foods_info.l")
;;(get_foods_info_main)
(get-lbox-coords)
(setq radi_list (list 0 0))
(setq name_list (list "tomato" "tomato"))
(setq box_list (list (list 28.8451 24.9597 36.2208) (list 28.8451 24.9597 36.2208)))
(setq pos_list (list #f(308.31 24.7863 -73.9793) #f(308.31 24.7863 -73.9793)))

;; ----ここに情報を手入力する-----
(setq pre_name_list (list "fried_chicken" "rolled_egg" "broccoli" "octopus_wiener" "broccoli"))
(setq pre_box_list (list (list 45.3886 45.2212 27.5869) (list 68.0041 37.0235 20.7607) (list 35.6604 37.4197 33.5302) (list 28.8451 24.9597 36.2208) (list 35.6604 37.4197 33.5302)))
(setq pre_radi_list (list 0 0 0 0 0))
(setq pre_pos_list (list 0 0 0 0 0))
;; ------------------------------
;;"broccoli" (list 35.1378 48.657 42.2121)
(setq plen (length pre_name_list))

(dotimes (i (length name_list))
  (setq pre_name_list (append pre_name_list (list (elt name_list i))))
  (setq pre_box_list (list-insert (elt box_list i) (length pre_box_list) pre_box_list))
  (setq pre_radi_list (append pre_radi_list (list (elt radi_list i))))
  (setq pre_pos_list (list-insert (elt pos_list i) (length pre_pos_list) pre_pos_list))
  )

(setq name_list pre_name_list)
(setq box_list pre_box_list)
(setq pos_list pre_pos_list)
(setq radi_list pre_radi_list)

(format t "name_list ~A~%" name_list)
(format t "pos_list ~A~%" pos_list)
(format t "box_list ~A~%" box_list)
(format t "radi_list ~A~%" radi_list)

;;publish food info and lunchbox info
(load "info_publisher.l")
(info_publisher_main)

;;Calculate stuff posiion using stuff_by_GA.py

;;subscribe result of GA caluculation
(load "result_subscriber.l")
(result_subscriber_main)

(load "catch_and_place.l")
(setq lbox_pos (float-vector (/ (+ *front* back) 2) (/ (+ *left* right) 2) 0))
(setq *tong-offset* #f(-20.23 -114.702 90.2251))
(setq *chopstick-offset* #f(-12.624 18.437 84.2388))
(setq *chopstick-offset-r* #f(-25.065 34.9855 84.2388)) ;;100.135))
(format t "- - - CATCH AND PLACE - - -~%")
(place-init-pose)
(pub-lbox-coords)
(sub-img 2)
(dotimes (i (length order_list))
  (when (= i 4)
    (format t "SET UP~%")
    (ros::duration-sleep 10))
  (place-init-pose)
  (setq num (elt order_list i))
  (format t "num : ~A~%" num)
  (when (> (elt (elt place_list num) 0) 0)
    (setq target_name (elt name_list num))
    (setq food-size (elt box_list num))
    (format t "target : ~A~%" target_name)
    (format t "size : ~A place : ~A~%" food-size (elt place_list num))
    (if (or (> (elt food-size 0) 50) (> (elt food-size 1) 50))
	(progn
	  (if (or (> (elt food-size 0) 60) (> (elt food-size 1) 60))
	      (open-tongs 2)
	    (open-tongs 1)))
      (open-tongs 0))
    (sub-img 0)
    (if (= (elt radi_list num) 0)
	(setq offset (v+ *tong-offset* #f(5 0 0))) ;;tong offset
      ;;(setq offset *tong-offset*)
      (setq offset (v+ *tong-offset* #f(0 15 0))) ;;内側め
      )
    (setq fail_catch_flag nil)
    (setq pull_count 0)
    (setq arrange_count 0)
    ;;実験用：トマト以外は手動で置く
    (if (string= target_name "tomato")
	(progn
	  (pick-food (elt pos_list num) (elt radi_list num) offset)
	  (place_tomato (elt place_list num) food-size))
      (progn
	(format t "PLEASE PUT FOOD in 10 seconds~%")
	(ros::duration-sleep 1.0)
	(sub-img 1)
	(send (send pub_status_msg :header) :stamp (ros::time-now))
	(ros::subscribe "/get_placed_pos/output" hironx_tutorial::FoodPacking #'place_cb)
	(setq catch_flag nil)
	(until catch_flag
	       (ros::publish "/get_placed_pos/input" pub_status_msg)
	       (ros::spin-once)
	       (ros::sleep))
	(ros::unsubscribe "/get_placed_pos/output")
	(if (not (= 3D_width 0))
	    (pull_over (elt place_list num) diff_x diff_y 3D_width 3D_length food-size approach)
	  (setq fail_catch_flag t)))
      )
    (setq retry_count 0)
    (while (and fail_catch_flag (string= target_name "tomato"))
      ;; retry to catch
      (setq retry_count (+ retry_count 1))
      (if (= retry_count 1)
	  (setq offset (v+ offset #f(0 0 -5)))
	(if (= retry_count 2)
	    (setq offset (v+ offset #f(0 0 10)))))
      (setq fail_catch_flag nil)
      (if (or (> (elt food-size 0) 50) (> (elt food-size 1) 50))
	  (progn
	    (if (or (> (elt food-size 0) 60) (> (elt food-size 1) 60))
		(open-tongs 2)
	      (open-tongs 1)))
	(open-tongs 0))
      (pick-food (elt pos_list num) (elt radi_list num) offset)
      (if (string= target_name "tomato")
	  (place_tomato (elt place_list num) food-size)
	(place-food (elt place_list num) target_name food-size)))
    )
  )

