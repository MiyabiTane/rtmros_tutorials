#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(hironxjsk-init)
(objects (list *hironxjsk*))

;;Rostopic
(ros::load-ros-package "jsk_recognition_msgs")
(ros::load-ros-package "geometry_msgs")
(setq *pub-topic* "/head_camera/rgb/image_rect_color/screenpoint")
(setq *sub-topic* "/pointcloud_screenpoint_nodelet/output_point")
(setq *camera-frame* "head_camera_rgb_optical_frame")


(defvar *coral-food-rects* "edgetpu_object_detector/output/rects")
(defvar *coral-food-class* "edgetpu_object_detector/output/class")

;;use tf
(unless (boundp '*tfl*)
  (setq *tfl* (instance ros::transform-listener :init)))


;; Look at table
(defun look-table ()
  (send *hironxjsk* :reset-manip-pose)
  (send *hironxjsk* :head :look-at
        (send (send (send *hironxjsk* :torso :end-coords :copy-worldcoords)
                    :translate #f(750 0 0)) :worldpos))
  (send *irtviewer* :draw-objects)  ;; Only for display
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1000)
  (send *ri* :wait-interpolation))


;;Only for draw
(defun draw (coords)
  (setq *target-food* (make-cube 80 80 80))
  (send *target-food* :locate coords)
  (send *target-food* :set-color :salmon)
  (objects *target-food*)
  )


;;make center-coords and publish
(defun publish-center-coords (*food-rects*)
  (format t "publish food-coords~%")
  (setq x (send *food-rects* :x))
  (setq y (send *food-rects* :y))
  (setq width (send *food-rects* :width))
  (setq height (send *food-rects* :height))
  ;;publish
  (let* ((x (+ x (/ width 2)))
	 (y (+ y (/ height 2)))
	 (pub-msg (instance geometry_msgs::Pointstamped :init)))
    (format t "x y ~A ~A ~%" x y)
    (send (send pub-msg :point) :x x)
    (send (send pub-msg :point) :y y)
    (send (send pub-msg :point) :z 0)
    (ros::publish *pub-topic* pub-msg)
    ))


;;Subscribe Coral and Publish
(defun coral_cb (*target-name*)
  (setq *food-rects* nil)
  (while (eql *food-rects* nil)
    (format t "coral_cb~%")
    (ros::duration-sleep 1.0)
    (setq *food-name-msg* (one-shot-subscribe *coral-food-class*
					      jsk_recognition_msgs::ClassificationResult
					      :after-stamp (ros::time-now)))
    (setq *food-rects-msg* (one-shot-subscribe *coral-food-rects*
					       jsk_recognition_msgs::RectArray
					       :after-stamp (ros::time-now)))
    ;;ほぼ同じタイミングのデータだが、内容がずれてしまった時は採用しない
    (when (= (length (send *food-rects-msg* :rects)) (length (send *food-name-msg* :label_names)))
      (format t "got data from coral~%")
      (setq index 0)
      (dolist (str (send *food-name-msg* :label_names))
	(format t "find ~A~%" str)
	(if (string-equal str *target-name*)
	    (progn
	      (format t "find target-food index is ~A~%" index)
	      (setq *food-rects* (elt (send *food-rects-msg* :rects) index))))
	(setq index (+ index 1)))
      ))
  ;;publish
  (publish-center-coords *food-rects*)
 )


;;screen-point CallBack
(defun screen_point_cb (msg)
  (format t "screen_point_cb~%")
  ;;ros::tf-point->coords *camera-frame*座標を"WAIST"座標に変換
  (let ((waist-to-camera (send *tfl* :lookup-transform "WAIST" *camera-frame* (ros::time 0)))
	(coords (make-coords :pos (ros::tf-point->pos (send msg :point)))))
    (send coords :transform waist-to-camera :world)
    (format t "food coords ~A~%" (send coords :pos))
    (draw (send coords :pos)))
  ;;(ros::exit)
  ;;(ros::unsubscribe *sub-topic*)
  )


(look-table)
(ros::roseus "screenpoint")
(ros::advertise *pub-topic* geometry_msgs::PointStamped 1)
(ros::subscribe *sub-topic* geometry_msgs::PointStamped #'screen_point_cb)
(coral_cb "tv")
(coral_cb "sports ball")


(while (ros::ok)
  (ros::spin-once)
  (ros::sleep))


