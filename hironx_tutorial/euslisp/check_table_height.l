#!/usr/bin/env roseus

(ros::load-ros-package "jsk_recognition_msgs")

(defun rwrench_cb (msg)
  (setq torque_data (send (send msg :wrench) :torque))
  (setq torque_z (send torque_data :z))
  ;;(format t "torque ~A~%" torque_z)
  (setq cb_flag t)
  (if (> torque_z 0)
      (setq loop_flag nil))
  )


(defun get-z-coords ()
  (init-pose-20201030)
  (open-hand)
  (setq *hiro-pos* (send (send (send *hironxjsk* :rarm :end-coords) :copy-worldcoords) :pos))
  (setq move-z (- (+ *G-z* 105) (elt *hiro-pos* 2)))
  (send *hironxjsk* :rarm :move-end-pos (float-vector 0 -170 move-z) :world)
  (send-robot)
  (setq moved_count 0)
  (setq loop_flag t)
  (setq cb_flag nil)
  (while loop_flag
    (when cb_flag
      (format t "right-torque ~A~%" torque_z)
      (send *hironxjsk* :rarm :move-end-pos #f(0 0 -5) :world)
      (setq moved_count (+ moved_count 1))
      (send-robot)
      (setq cb_flag nil)
      )
    (ros::spin-once)
    (ros::sleep)
    )
  ;;Retry
  (if (< moved_count 2)
      (get-z-coords))
  (setq *catch-z* torque_z)
  (format t "catch-z ~A~%" *catch-z*)
  (ros::unsubscribe "/rhsendor")
  ;;(init-pose-20201030)
  )

(defun check_table_height_main ()
  (format t "- - - CHECK TABLE HEIGHT - - -~%")
  (ros::subscribe "/rhsensor" geometry_msgs::WrenchStamped #'rwrench_cb)
  (get-z-coords)
  (init-pose-20201030)
  )

