#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(hironxjsk-init)
(objects (list *hironxjsk*))

;;Rostopic
(ros::load-ros-package "jsk_recognition_msgs")
(ros::load-ros-package "geometry_msgs")
(setq *pub-topic* "/head_camera/rgb/image_rect_color/screenpoint")
(setq *sub-topic* "/pointcloud_screenpoint_nodelet/output_point")

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


;;Subscribe Coral and Publish
(defun coral_cb (msg)
  (format t "coral_cb~%")
  (when *coral-food-rects*
    (setq *food-rects* (elt (send msg :rects) 0))
    (setq x (send *food-rects* :x))
    (setq y (send *food-rects* :y))
    ;;publish
    (let* ((x (send *food-rects* :x))
	   (y (send *food-rects* :y))
	   (pub-msg (instance geometry_msgs::Pointstamped :init)))
      (format t "x y ~A ~A ~%" x y)
      (send (send pub-msg :point) :x x)
      (send (send pub-msg :point) :y y)
      (send (send pub-msg :point) :z 0)
      (ros::publish *pub-topic* pub-msg)
      ))
  )


;;CallBack
(defun screen_point_cb (msg)
  (format t "screen_point_cb~%")
  (setq *food-x* (send (send msg :point) :x))
  (setq *food-y* (send (send msg :point) :y))
  (setq *food-z* (send (send msg :point) :z))
  (format t "food coords is ~A ~A ~A ~%" *food-x* *food-y* *food-z*)
  (ros::exit)
  )


(look-table)
(ros::roseus "screenpoint")
(ros::advertise *pub-topic* geometry_msgs::PointStamped 1)
(ros::subscribe *sub-topic* geometry_msgs::PointStamped #'screen_point_cb)
(ros::subscribe *coral-food-rects* jsk_recognition_msgs::RectArray #'coral_cb)

(while (ros::ok)
  (ros::spin-once)
  (ros::sleep))


