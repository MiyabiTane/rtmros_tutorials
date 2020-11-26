#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(hironxjsk-init)
(objects (list *hironxjsk*))

;;init
(ros::roseus "hiro_lunchbox")

(ros::load-ros-package "jsk_recognition_msgs")

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
  (send-robot)
   ;;look table
  (send *hironxjsk* :head :look-at
        (send (send (send *hironxjsk* :torso :end-coords :copy-worldcoords)
                    :translate #f(750 0 0)) :worldpos))
  (send *hironxjsk* :head :look-at #f(500 0 0))
  (send-robot)
  ;;open hand
  (send *hironxjsk* :hand-angle-vector :lhand #f(20.449 -20.449 -20.449 20.449))
  (send-left-hand 2.0)
  (send *hironxjsk* :hand-angle-vector :rhand #f(20.449 -20.449 -20.449 20.449))
  (send-right-hand 2.0)
  )


(ros::load-ros-package "jsk_recognition_msgs")
(setq *sub-topic* "segmentation_decomposer_ssd/boxes")

(defun find-ssd-box ()
 (setq find_flag nil)
  (while (eql find_flag nil)
    (setq *bbox-msg* (one-shot-subscribe *sub-topic*
					 jsk_recognition_msgs::BoundingBoxArray
					 :after-stamp (ros::time-now)))
    (when (> (length (send *bbox-msg* :boxes)) 0)
      (format t "find box~%")
      (setq find_flag t)
      (setq bbox (elt (send *bbox-msg* :boxes) 0))
      (setq cascoords (send (send *tfl* :lookup-transform "WAIST" (send bbox :header :frame_id) (ros::time 0))
			    :transform (ros::tf-pose->coords (send bbox :pose))))
      (setq *coords* (send cascoords :worldpos))
      (format t "coords ~A~%" *coords*)
      )
    )
  )

(defun move-to ()
  (setq *hiro-pose* (send (send *hironxjsk* :rarm :end-coords) :copy-worldcoords))
  (setq *move* (v- *coords* (send *hiro-pose* :pos)))
  (send *hiro-pose* :translate *move* :world)
  (send *hironxjsk* :rarm :inverse-kinematics *hiro-pose*)
  ;;(format t "target-pose ~A~%" *hiro-pose*)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 100) :world)
  (send-robot)
  )

;;---------------grasp force-----------------
(defun move-to-right-hand ()
  (setq *tong-offset* #f(-39.159 -114.508 100.161))
  (setq *coords* (v+ *coords* *tong-offset*))
  (setq *hiro-pose* (send (send *hironxjsk* :rarm :end-coords) :copy-worldcoords))
  (setq *move* (v- *coords* (send *hiro-pose* :pos)))
  (send *hiro-pose* :translate *move* :world)
  (send *hironxjsk* :rarm :inverse-kinematics *hiro-pose*)
  ;;(format t "target-pose ~A~%" *hiro-pose*)
  ;;(send *hironxjsk* :rarm :move-end-pos #f(0 0 100) :world)
  (send-robot)
  )

(defun grasp-force ()
  (setq close #f(-5 5 5 -5))
  (setq diff 0)
  (while (< diff 2.0)
    (setq prev_hand (send *ri* :hand-angle-vector :rhand))
    (send *hironxjsk* :hand-angle-vector :rhand (v+ prev_hand close))
    (send-right-hand 2.0)
    (setq cur_hand (send *ri* :hand-angle-vector :rhand))
    (setq diff (norm (v- (v- cur_hand prev_hand) close)))
    (format t "diff ~A~%" diff)
    (when (< (elt (send *hironxjsk* :hand-angle-vector :rhand) 0) -16)
      (format t "enough~%")
      (setq diff 3))
    (send-right-hand 2.0)
    )
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 50) :world)
  (send-robot)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 -50) :world)
  (send-robot)
  (send *hironxjsk* :hand-angle-vector :rhand #f(20.449 -20.449 -20.449 20.449))
  (send-right-hand 2.0)
  )

(init-pose-20201030)
;;(find-ssd-box)

;;right-hand (rotate 90)
;;#f(-39.159 -114.508 100.161) hand-pos - real-pos
