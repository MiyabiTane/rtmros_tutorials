#!/usr/bin/env roseus

(ros::load-ros-package "jsk_recognition_msgs")

(defun rwrench_cb (msg)
  (setq torque_data (send (send msg :wrench) :torque))
  (setq force_data (send (send msg :wrench) :force))
  (unless prev_torque
    (setq prev_torque (send (send msg :wrench) :torque)))
  (unless prev_force
    (setq prev_force (send (send msg :wrench) :force)))
  (setq cur_torque torque_data)
  (setq cur_force force_data)
  (setq torque_x_diff (- (send cur_torque :x) (send prev_torque :x)))
  (setq force_z_diff (- (send cur_force :z) (send prev_force :z)))
  (when (or (> torque_x_diff 0.15)) (> force_z_diff 0.15)
	(setq loop_flag nil))
  (setq prev_torque cur_torque)
  (setq prev_force cur_force)
  (setq cb_flag t)
  )

(defun get-z-coords ()
  (setq prev_torque nil)
  (setq prev_force nil)
  (setq loop_flag t)
  (setq cb_flag nil)
  (while loop_flag
    (if stop-move-flag
	(setq loop_flag nil))
    (ros::spin-once)
    (ros::sleep)
    (when cb_flag
      (format t "torque diff ~A force_diff ~A~%" torque_x_diff force_z_diff)
      (when loop_flag
	(send *hironxjsk* :rarm :move-end-pos #f(0 0 -3) :world)
	(send-robot))
      (setq cb_flag nil)
      )
    )
  (format t "finish loop~%")
  (setq *catch-z* (elt (send (send (send *hironxjsk* :rarm :end-coords) :copy-worldcoords) :pos) 2))
  (setq *catch-z* (+ *catch-z* 2))
  (format t "catch-z ~A~%" *catch-z*)
  )

(defun check_table_height_main ()
  (format t "- - - CHECK TABLE HEIGHT - - -~%")
  (ros::subscribe "/rhsensor" geometry_msgs::WrenchStamped #'rwrench_cb)
  (init-pose-20201030)
  (open-hand)
  (setq *hiro-pos* (send (send (send *hironxjsk* :rarm :end-coords) :copy-worldcoords) :pos))
  (setq move-z (- (+ *G-z* 80) (elt *hiro-pos* 2)))
  (send *hironxjsk* :rarm :move-end-pos (float-vector 120 -100 move-z) :world)
  (send-robot)
  (get-z-coords)
  (ros::unsubscribe "/rhsendor")
  (init-pose-20201030)
  )

