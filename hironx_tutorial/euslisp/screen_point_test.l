#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(hironxjsk-init)
(objects (list *hironxjsk*))

;;Rostopic
(ros::load-ros-package "jsk_recognition_msgs")
(setq *ray_srv* "/pointcloud_screenpoint_nodelet/screen_to_point")

(defvar *coral-food-rects* "edgetpu_object_detector/output/rects")


;; Look at table
(defun look-table ()
  (send *hironxjsk* :reset-manip-pose)
  (send *hironxjsk* :head :look-at
        (send (send (send *hironxjsk* :torso :end-coords :copy-worldcoords)
                    :translate #f(750 0 0)) :worldpos))
  (send *irtviewer* :draw-objects)  ;; Only for display
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1000)
  (send *ri* :wait-interpolation))


;;CallBack
(defun screen_point_cb (msg)
  (format t "call back~%")
  (when *coral-food-rects*
    (setq *food-rects* (elt (send msg :rects) 0))
    (let* ((x (send *food-rects* :x))
	   (y (send *food-rects* :y))
	   (req (instance jsk_recognition_msgs::TransformScreenpointRequest :init
			  :x x :y y))
	   res)
      (format t "x y ~A ~A~%" x y)
      (format t ";;wait for service~%")
      (ros::wait-for-service *ray_srv*)
      (setq res (ros::service-call *ray_srv* req))
      (setq *food-x* (send (send res :vector) :x))
      (setq *food-y* (send (send res :vector) :y))
      (setq *food-z* (send (send res :vector) :z))
      (format t "food coords is ~A ~A ~A ~%" *food-x* *food-y* *food-z*)
      )))


(look-table)
(ros::roseus "screenpoint")
(ros::subscribe *coral-food-rects* jsk_recognition_msgs::RectArray #'screen_point_cb)
(ros::rate 10)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep))

(ros::exit)
