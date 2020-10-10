#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(hironxjsk-init)
(objects (list *hironxjsk*))

;use tf
(unless (boundp '*tfl*)
  (setq *tfl* (instance ros::transform-listener :init)))

(ros::load-ros-package "jsk_recognition_msgs")

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
      (setq size (ros::tf-point->pos (send bbox :dimensions)))
      (format t "size ~A~%" size)
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
  (setq x_2 (send force_data :x))
  (setq y_2 (send force_data :y))
  (setq z_2 (send force_data :z))
  (setq norm_2 (+ x_2 y_2 z_2))
  )

(defun get_range()
  ;;know right edge
  (setq norm_2 10)
  (while (< norm_2 50)
    (format t "norm_2 is ~A~%" norm_2)
    (send *hironxjsk* :larm :move-end-pos #f(0 -5 0))
    (send-robot)
    )
  (setq hiro-hand-pos (elt (send (send *hironxjsk* :larm :end-coords) :copy-worldcoords) 0))
  (setq *right-edge* (v+ hiro-hand-pos #f(50 50 -150)))
  (format t "right edge is ~%")
  )

(init-pose-20201001)
(defun main()
  (setq *coords* #f(0 -100 0))
  (while (< (elt *coords* 1) 0)
    (find-box)
    )
  (move_to)
  (get_range)
  )
(ros::subscribe *sub-topic* geometry_msgs::WrenchStamped #'wrench_cb)
