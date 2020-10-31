#!/usr/bin/env roseus

#|
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(hironxjsk-init)
(objects (list *hironxjsk*))

;;init
(ros::roseus "hiro_lunchbox")
|#

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

(defun init-pose-20201030 ()
  (send *hironxjsk* :reset-manip-pose)
  (send *hironxjsk* :larm_joint4 :joint-angle -35)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 100) :world)
  (send *hironxjsk* :rarm :move-end-pos #f(50 0 150) :world)
  (send *hironxjsk* :rarm_joint5 :joint-angle 100)
  (send *hironxjsk* :rarm_joint3 :joint-angle 35)
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

(defun rwrench_cb (msg)
  (setq torque_data (send (send msg :wrench) :torque))
  (setq torque_z (send torque_data :z))
  ;;(format t "torque ~A~%" torque_z)
  (setq cb_flag t)
  (if (or (> torque_z 0) (< torque_z -0.15))
      (setq loop_flag nil))
  )
  
  
(defun get-z-coords ()
  (init-pose-20201030)
  (send *hironxjsk* :rarm :move-end-pos #f(0 -150 -100) :world)
  (send-robot)
  (setq loop_flag t)
  (setq cb_flag nil)
  (while loop_flag
    (when cb_flag
      (format t "right-torque ~A~%" torque_z)
      (send *hironxjsk* :rarm :move-end-pos #f(0 0 -5) :world)
      (send-robot)
      (setq cb_flag nil)
      )
    (ros::spin-once)
    (ros::sleep)
    )
  (setq *catch-z* (+ torque_z 10))
  (format t "catch-z ~A~%" *catch-z*)
  ;;(init-pose-20201030)
  )

(ros::subscribe "/rhsensor" geometry_msgs::WrenchStamped #'rwrench_cb)
(get-z-coords)
      
