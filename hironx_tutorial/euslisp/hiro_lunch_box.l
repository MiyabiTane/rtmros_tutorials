#!/usr/bin/env roseus

;; You need to run stuff_by_GA.py
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
(defvar *bounding-box-list* "/segmentation_decomposer/boxes")

;;keep info
(setq name_list '())
(setq pos_list '())
(setq box_list '())

;;use tf
(unless (boundp '*tfl*)
  (setq *tfl* (instance ros::transform-listener :init)))

;;init
(ros::roseus "hiro_lunchbox")

;;get food position and size
(load "get_foods_info.l")
(format t "name_list ~A~%" name_list)
(format t "pos_list ~A~%" pos_list)
(format t "box_list ~A~%" box_list)
;;get lunchbox position and size
(load "recognize_lunchbox.l")
(format t "left ~A front ~A~%" *left* *front*)
(format t "size GA width ~A length ~A~%" *length* *width*)
;;publish food info and lunchbox info
(load "info_publisher.l")
;;Calculate stuff posiion using stuff_by_GA.py
(load "result_subscriber.l")

;;------------------------------------------------------------------------------------------------

(defun find_best_stuff (name_list pos_list box_list)
  ;;To Do
  ;;lunchboxの座標は0とかにしとく（使わないけどリストのインデックスは対応させたい）
  ;;(setq place_list  (#f(650 0 20) #f(650 0 20) ... ))
  (setq place_list '())
  )


;;To move robot
(defvar time-scale 1.0)

(defun send-robot ()
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (sleep-time-scale 800)
  (irt))

(defun send-robot-faster ()
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (sleep-time-scale 500)
  (irt))

(defun sleep-time-scale (tm)
  (unix:usleep (truncate (* 1000 time-scale tm))))

(defun irt () (send *irtviewer* :draw-objects))

(defun open-hand (which_hand)
  ;;0:left 1:right
  (if (= which_hand 0)
      (send *ri* :stop-grasp :larm)
    (send *ri* :stop-grasp :rarm))
  (send *ri* :wait-interpolation))

(defun close-hand (which_hand)
  (if (= which_hand 0)
      (send *ri* :start-grasp :larm)
    (send *ri* :start-grasp :rarm))
  (send *ri* :wait-interpolation))


;;List
;;pos_list, place_list, name_list, (box_list) 

;;elt pos 1 > 0なら左手で掴む
(defun grasp-food-l (pos)
  (send *hironxjsk* :larm :inverse-kinematcs (make-coords :pos (v+ pos #f(0 0 100))))
  (send-robot)
  (open-hand 0)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 -100) :world)
  (send-robot)
  (close-hand 0)
  (ros::duration-sleep 2.0)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 100) :world)
  (send-robot))


(defun grasp-food-r (pos)
  (send *hironxjsk* :rarm :inverse-kinematics (make-coords :pos (v+ pos #f(0 0 100))))
  (send-robot)
  (open-hand 1)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 -100) :world)
  (close-hand 1)
  (ros::duration-sleep 2.0)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 100) :world)
  (send-robot))
  

(defun place-food-l (place)
  (send *hironxjsk* :larm :inverse-kinematics (make-coords :pos (v+ place #f(0 0 100))))
  (send-robot)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 -100) :world)
  (send-robot)
  (open-hand 0)
  (send *hironxjsk* :larm :move-pos #f(0 0 100) :world)
  (send-robot))


(defun place-food-r (place)
  (send *hironxjsk* :rarm :inverse-kinematics (make-coords :pos (v+ place #f(0 0 100))))r
  (send-robot)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 -100) :world)
  (send-robot)
  (open-hand 1)
  (send *hironxjsk* :rarm :move-pos #f(0 0 100) :world)
  (send-robot))


(defun stuff-all (name_list pos_list place_list)
  (dotimes (i *leng*)
    (setq target-food (elt name_list i))
    (format t "try to stuff ~A~%" target-food)
    (setq target-pos (elt pos_list i))
    (format t "target-pos ~A~%" target-pos)
    ;;(setq target-place (elt place_list i))
    (setq target-place #f(650 0 20))
    (unless (string-equal target-food "tv") ;;tv -> lunchbox
      (if (> (elt target-pos 1) 0)
	  (progn
	    (grasp-food-l target-pos)
	    (place-food-l target-place))
      (progn
	(grasp-food-r target-pos)
	(place-food-r target-place)))
      )))

;;--------------------------------------------------------------------------------------------------------



  
  
  
  
