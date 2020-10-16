#!/usr/bin/env roseus

;;(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
;;(hironxjsk-init)
;;(objects (list *hironxjsk*))

;use tf
;;(unless (boundp '*tfl*)
;;  (setq *tfl* (instance ros::transform-listener :init)))

;;(ros::load-ros-package "jsk_recognition_msgs")

(defvar *bbox-list* "/segmentation_decomposer/boxes")

(setq *sub-topic* "/lhsensor")

;;diag-chopsticks
(defun init-pose-20201001 ()
  (send *hironxjsk* :reset-manip-pose)
  (send *hironxjsk* :larm_joint4 :joint-angle -35)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 100) :world)
  (send *hironxjsk* :rarm_joint1 :joint-angle 20) 
  (send *hironxjsk* :rarm :move-end-pos #f(150 0 300))
  (send-robot)
  ;;look table
  (send *hironxjsk* :head :look-at
        (send (send (send *hironxjsk* :torso :end-coords :copy-worldcoords)
                    :translate #f(750 0 0)) :worldpos))
  (send *hironxjsk* :head :look-at #f(600 0 0))
  (send-robot)
  ;;open hand
  (send *hironxjsk* :hand-angle-vector :lhand #f(20.449 -20.449 -20.449 20.449))
  (send-left-hand 2.0)
  (send *hironxjsk* :hand-angle-vector :rhand #f(20.449 -20.449 -20.449 20.449))
  (send-right-hand 2.0)
  )


;;to move robot
(defun send-robot ()
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

(defun send-left-hand (sleep-time)
  (send *ri* :hand-angle-vector :lhand (send *hironxjsk* :hand-angle-vector :lhand) 2000)
  (ros::duration-sleep sleep-time)
  )

(defun send-right-hand (sleep-time)
  (send *ri* :hand-angle-vector :rhand (send *hironxjsk* :hand-angle-vector :rhand) 2000)
  (ros::duration-sleep sleep-time)
  )


;;get bounding box (= lunchbox)
(defun find-box ()
  (setq find_flag nil)
  (while (eql find_flag nil)
    (setq *bbox-msg* (one-shot-subscribe *bbox-list*
					 jsk_recognition_msgs::BoundingBoxArray
					 :after-stamp (ros::time-now)))
    (when (> (length (send *bbox-msg* :boxes)) 0)
      (format t "find box~%")
      (setq find_flag 1)
      (setq bbox (elt (send *bbox-msg* :boxes) 0))
      (setq cascoords (send (send *tfl* :lookup-transform "WAIST" (send bbox :header :frame_id) (ros::time 0))
			    :transform (ros::tf-pose->coords (send bbox :pose))))
      (setq *coords* (send cascoords :worldpos))
      (format t "coords ~A~%" *coords*)
      (setq *G-z* (elt *coords* 2))
      (setq size (ros::tf-point->pos (send bbox :dimensions)))
      (setq *height* (elt size 2))
      (format t "height ~A~%" *height*)
      ))
  )

(defun move_to ()
  ;;close chopsticks 
  (send *hironxjsk* :hand-angle-vector :lhand #f(-10.499 10.499 10.499 -10.499))
  (send-left-hand 1.0)
  ;;length of chopsticks
  (setq *off-set* (float-vector -50 -50 150)) ;;To Do x,y -> use box size
  (setq *start-pos* (v+ *off-set* *coords*))
  (setq *hiro-pose* (send (send *hironxjsk* :larm :end-coords) :copy-worldcoords))
  (setq *move* (v- *start-pos* (send *hiro-pose* :pos))) 
  (send *hiro-pose* :translate *move* :world)
  (send *hironxjsk* :larm 
	:inverse-kinematics *hiro-pose*)
  (send-robot)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 -70) :world)
  (send-robot)
  )

(defun wrench_cb(msg)
  (setq force_data (send (send msg :wrench) :force))
  (setq force_vector (float-vector (send force_data :x) (send force_data :y) (send force_data :z)))
  (setq force_norm (norm force_vector))
  (setq torque_data (send (send msg :wrench) :torque))
  (setq torque_vector (float-vector (send torque_data :x) (send torque_data :y) (send torque_data :z)))
  (setq torque_norm (norm torque_vector))
  (setq got_info 1) 
  )

(defun move_front_back(thresh move-vector *edge*)
  (setq prev_norm 0)
  (setq torque_norm 0)
  (setq flag 0)
  (while (eql flag 0) 
    (setq got_info 0)
    (while (eql got_info 0)
      (ros::spin-once)
      (ros::sleep)
      )
    (if (eql prev_norm 0)
	(setq prev_norm torque_norm))
    (format t "diff ~A~%" (- torque_norm prev_norm))
    (when (> thresh 0)
	(if (> (- torque_norm prev_norm) thresh)
	    (setq flag 1)))
    (when (< thresh 0)
	(if (< (- torque_norm prev_norm) thresh)
	    (setq flag 1)))
    ;;(format t "torque_vector is ~A ~A ~%" torque_vector torque_norm)
    (send *hironxjsk* :larm :move-end-pos move-vector :world)
    (send-robot)
    )
  (setq hiro-hand-pos (send (send (send *hironxjsk* :larm :end-coords) :copy-worldcoords) :pos))
  (setq *edge* (v+ hiro-hand-pos (scale -1 *off-set*)))
  (format t "edge is ~A~%" *edge*)
  (send *hironxjsk* :larm :move-end-pos (scale -2 move-vector))
  (send-robot)
  )

(defun move_left_right(thresh move-vector *edge*)
  ;;know right edge
  (setq prev_hand 0)
  (setq flag 0)
  (while (eql flag 0)
    (setq cur_hand (elt (send *ri* :hand-angle-vector :lhand) 0))
    (if (eql prev_hand 0)
	(setq prev_hand cur_hand))
    (format t "diff ~A~%" (- prev_hand cur_hand))
    (when (> thresh 0)
	(if (> (- torque_norm prev_norm) thresh)
	    (setq flag 1)))
    (when (< thresh 0)
	(if (< (- torque_norm prev_norm) thresh)
	    (setq flag 1)))
    ;;(format t "hand-vector is ~A~%" cur_hand)
    (send *hironxjsk* :larm :move-end-pos move-vector :world)
    (send-robot)
    )
  (setq hiro-hand-pos (send (send (send *hironxjsk* :larm :end-coords) :copy-worldcoords) :pos))
  (setq *edge* (v+ hiro-hand-pos (scale -1 *offset*)))
  (format t "edge is ~A~%" *edge*)
  (send *hironxjsk* :larm :move-end-pos (scale -2 move-vector))
  (send-robot)
  )

(defun get_range()
  (move_left_right 1 #f(0 -5 0) *right-edge*) ;;right
  (move_front_back 0.1 #f(5 0 0) *back-edge*) ;;back
  (move_left_right -0.1 #f(0 5 0) *left-edge*) ;;left
  (move_front_back -0.1 #f(-5 0 0) *front-edge*) ;;front
  )

(defun box_vis() ;;for debug
  (setq right (elt *right-edge* 1))
  (setq back (elt *back-edge* 0))
  (setq *left* (elt *left-edge* 1))
  (setq *front* (elt *front-edge* 0))
  (setq *width* (- left right))
  (setq *length* (- back front))
  (setq *G-y* (/ (+ right left) 2))
  (setq *G-x* (/ (+ back front) 2))
  (setq l_box (make-cube *length* *width* *height*))
  (send l_box :locate (float-vector *G-y* *G-x* *G-z*))
  (objects l_box) ;;visualize
  )

(defun main()
  (init-pose-20201001)
  ;; To Do 腕を広げてよける
  (setq *coords* #f(0 -100 0))
  (while (< (elt *coords* 1) 0)
    (find-box)
    )
  (move_to)
  (get_range)
  (box_vis)
  (init-pose-20201001)
  )

(ros::subscribe *sub-topic* geometry_msgs::WrenchStamped #'wrench_cb)
(main)
