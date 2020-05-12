#!/usr/bin/env roseus

(ros::roseus "simple-screen-point-test")
(ros::load-ros-package "jsk_recognition_msgs")
(setq *ray_srv* "/pointcloud_screenpoint_nodelet/screen_to_point")

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(hironxjsk-init)

(defun look-table ()
  (send *hironxjsk* :reset-manip-pose)
  (send *hironxjsk* :head :look-at
        (send (send (send *hironxjsk* :torso :end-coords :copy-worldcoords)
                    :translate #f(750 0 0)) :worldpos))
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1000)
  (send *ri* :wait-interpolation))

(look-table)

(let* ((x 240)
	   (y 320)
	   (req (instance jsk_recognition_msgs::TransformScreenpointRequest :init
                      :x x :y y))
	   res)
  (format t "x y ~A ~A~%" x y)
  (format t ";;wait for service~%")
  (ros::wait-for-service *ray_srv*)
  (setq res (ros::service-call *ray_srv* req))
  (format t "food coords is ~A ~A ~A ~%"
          (send (send res :vector) :x)
          (send (send res :vector) :y)
          (send (send res :vector) :z))
  )

(ros::exit)
