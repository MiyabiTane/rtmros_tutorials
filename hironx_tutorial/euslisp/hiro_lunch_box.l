#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(hironxjsk-init)
(objects (list *hironxjsk*))

;;Rostopic
(ros::load-ros-package "jsk_recognition_msgs")

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


(defun coral-cb1 (msg)
  (setq *food-name-msg* msg))

(defun coral-cb2 (msg)
  (setq *food-rects-msg* msg))
  
  

(defun get-msg (*target-name*) ;;*target-name* -> example "tv"
  (if *food-rects-msg*
      (progn
	;;(foramt t "time 1 ~A~%" (send (send *food-name-msg* :header) :stamp))
	;;(format t "time 2 ~A~%" (send (send *food-rects-msg* :header) :stamp))
	(setq *food-rects* nil)
	(setq index 0)
	(dolist (str (send *food-name-msg* :label_names))
	  (format t "find  ~A~%" str)
	  (if (string-equal str *target-name*)
	      (progn
		(format t "find-obj index is ~A~%" index)
		(setq *food-rects* (elt (send *food-rects-msg* :rects) index))))
	  (setq index (+ index 1)))
	))
  )
  

;;Coralの座標
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
(defun get-3d-coords ()
  (let* ((x *coral-food-x*)
	 (y *coral-food-y*)
	 (req (instance jsk_recognition_msgs::TransformScreenpointRequest :init
			:x x :y y))
	 res)
    
    ;;(setq req (instance jsk_recognition_msgs::TransformScreenpointRequest :init))
    ;;(send req :x *coral-food-x*)
    ;;(send req :y *coral-food-y*)
    (format t ";; wait for *ray_srv*~%~%")
    (ros::wait-for-service *ray_srv*) 
    (setq res (ros::service-call *ray_srv* req))
    (setq *food-x* (send (send res :vector) :x))
    (setq *food-y* (send (send res :vector) :y))
    (setq *food-z* (send (send res :vector) :z))
    (format t "food coords is ~A ~A ~A ~%" *food-x* *food-y* *food-z*)
    ))


(defun main ()
  (setq *food-rects* nil)
  (while (eql *food-rects* nil)
    (when *coral-food-class*
      (get-msg "tv")))
  (get-food-coords)
  (get-3d-coords)
  ;;[ERROR] [1589097303.138212744, 146.253000000]: attempted to call service  /pointcloud_screenpoint_nodelet/screen_to_point, but failed 

  (ros::exit)
  )

(setq *food-rects-msg* nil)

(look-table)
(ros::roseus "get-3d-coords")
(setq *ray_srv* "/pointcloud_screenpoint_nodelet/screen_to_point")
;;To Do 同じ時刻のを取ってくる
(ros::subscribe *coral-food-class* jsk_recognition_msgs::ClassificationResult #'coral-cb1)
(ros::subscribe *coral-food-rects* jsk_recognition_msgs::RectArray #'coral-cb2)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep)
  (if *food-rects-msg*
      (main)))

