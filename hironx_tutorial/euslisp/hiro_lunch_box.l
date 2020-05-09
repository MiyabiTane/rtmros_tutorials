#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(hironxjsk-init)
(objects (list *hironxjsk*))

;;Rostopic
(ros::load-ros-package "jsk_recognition_msgs")
(ros::load-ros-package "jsk_pcl_ros")
(ros::load-ros-package "roseus")


(defvar *coral-food-class* "edgetpu_object_detector/output/class")
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


;;To Do
;;同じ時間のlabel_namesとrectsを参照して指定した食べ物の座標をとってくる
;;(defun get-msg (food-name)
;;  (setq *food-name* null)
;;  (while (not (food-name in *food-list*))

(look-table)

(defun get-msg ()
  (ros::duration-sleep 1.0)
  (setq *food-rects-msg* (one-shot-subscribe *coral-food-rects*
					     jsk_recognition_msgs::RectArray
					     :after-stamp (ros::time-now)))
  (setq *food-rects* (elt (send *food-rects-msg* :rects) 0))
  )
  

;;あくまでCoralの座標
(defun get-food-coords ()
  (setq x (send *food-rects* :x))
  (setq y (send *food-rects* :y))
  (setq width (send *food-rects* :width))
  (setq height (send *food-rects* :height))
  (setq *coral-food-x* (+ x (/ width 2)))
  (setq *coral-food-y* (+ y (/ height 2)))
  (format t "coral-coords ~A ~A ~%" *coral-food-x* *coral-food-y*)
  )
  
;;3次元上の座標獲得
(defun get-3d-coords (*coral-food-x* *coral-food-y*)
  (setq req (instance jsk_recognition_msgs::TransformScreenpointRequest :init))
  (send req :x *coral-food-x*)
  (send req :y *coral-food-y*)
  (setq res (ros::service-call "pointcloud_screenpoint/screen_to_point" req))
  (setq *food-x* (send (send res :point) :x))
  (setq *food-y* (send (send res :point) :y))
  (setq *food-z* (send (send res :point) :z))
  (format t "food coords is ~A ~A ~A ~%" *food-x* *food-y* *food-z*)
  )

(ros::roseus "get-3d-coords")
(format t ";; wait for pointcloud_screenpoint/screen_to_point~%~%")
(ros::wait-for-service "pointcloud_screenpoint/screen_to_point")

(defun main ()
  (get-msg)
  (when *food-rects*
    (get-food-coords)
    ;;(get-3d-coords *coral-food-x* *coral-food-y*)
    ;;[ERROR] [1589017913.486692063, 46.441000000]: attempted to call service  /pointcloud_screenpoint/screen_to_point, but failed 
    )
  (exit)
  )

(main)
(ros::spin)
  
