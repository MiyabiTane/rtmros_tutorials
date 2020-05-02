#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(hironxjsk-init)
(objects (list *hironxjsk*))

(ros::load-ros-package "jsk_recognition_msgs")


;;Rostopic
;;(defvar *bounding-box-list* "/segmentation_decomposer/boxes")
(defvar *bounding-box-list* "HSI_color_filter/boxes")

;;Send to real Robot
(defun send-robot ()
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (irt))

;;Open hand
(defun open-hand ()
  (send *ri* :stop-grasp :rarm)
  (send *ri* :wait-interpolation))

(defun open-hand-l ()
  (send *ri* :stop-grasp :larm)
  (send *ri* :wait-interpolation))

;;Close hand
(defun close-hand ()
  (send *ri* :start-grasp :rarm)
  (send *ri* :wait-interpolation))

(defun close-hand-l ()
  (send *ri* :start-grasp :larm)
  (send *ri* :wait-interpolation))


(defun irt ()
  (send *irtviewer* :draw-objects))

(defun tf-pose->coords (frame_id pose)
  (send (send *tfl* :lookup-transform "WAIST" frame_id (ros::time 0))
        :transform (ros::tf-pose->coords pose)))


;; Information of :boxes -> make coords
(defun bbox->cube (bbox)
  ;;to know box size
  (setq size (ros::tf-point->pos (send bbox :dimensions)))
  (setq *box-x* (elt size 0))
  (setq *box-y* (elt size 1))
  (setq *box-z* (elt size 2))
  (let* ((dims (ros::tf-point->pos (send bbox :dimensions)))
         (bx (make-cube (elt dims 0) (elt dims 1) (elt dims 2))))
    (send bx :newcoords
          (tf-pose->coords (send bbox :header :frame_id) (send bbox :pose)))
    bx))


;;Get the information from box
(defun get-box-coords ()
  (setq find-box 0)
  (ros::duration-sleep 1.0)
  (setq *bbox-msg* (one-shot-subscribe *bounding-box-list*
				jsk_recognition_msgs::BoundingBoxArray
				:after-stamp (ros::time-now)))
  (setq *target-bbox* (elt (send *bbox-msg* :boxes) 0)) ;;Box <- type:list
  ;;Get the shape and coords of the box
  (setq *target-cube* (bbox->cube *target-bbox*))
  (objects *target-cube*) ;;For view
  (format t "cube size ~A ~A ~A~%" *box-x* *box-y* *box-z*)
  (if (and (< *box-x* 65) (< *box-z* 45))
      (setq find-box 1))
  )

;;Choose target color
(defun color-set (num)
  (cond ((= num 0)
	 (progn ;;Red
	   (ros::set-dynamic-reconfigure-param "/HSI_color_filter/hsi_filter" "h_limit_max" :int 20)
	   (ros::set-dynamic-reconfigure-param "/HSI_color_filter/hsi_filter" "h_limit_min" :int -20)
	   (ros::set-dynamic-reconfigure-param "/HSI_color_filter/hsi_filter" "s_limit_max" :int 255)
	   (ros::set-dynamic-reconfigure-param "/HSI_color_filter/hsi_filter" "s_limit_min" :int 80)
	   (ros::set-dynamic-reconfigure-param "/HSI_color_filter/hsi_filter" "i_limit_max" :int 255)
	   (ros::set-dynamic-reconfigure-param "/HSI_color_filter/hsi_filter" "i_limit_min" :int 50)))
	((= num 1)
	 (progn ;;Blue
	   (progn ;;Red
	   (ros::set-dynamic-reconfigure-param "/HSI_color_filter/hsi_filter" "h_limit_max" :int -75)
	   (ros::set-dynamic-reconfigure-param "/HSI_color_filter/hsi_filter" "h_limit_min" :int -96)
	   (ros::set-dynamic-reconfigure-param "/HSI_color_filter/hsi_filter" "s_limit_max" :int 255)
	   (ros::set-dynamic-reconfigure-param "/HSI_color_filter/hsi_filter" "s_limit_min" :int 95)
	   (ros::set-dynamic-reconfigure-param "/HSI_color_filter/hsi_filter" "i_limit_max" :int 255)
	   (ros::set-dynamic-reconfigure-param "/HSI_color_filter/hsi_filter" "i_limit_min" :int 70))
	   ))
	((= num 2) ;;Green
	 (progn
	   (progn ;;Red
	   (ros::set-dynamic-reconfigure-param "/HSI_color_filter/hsi_filter" "h_limit_max" :int 100)
	   (ros::set-dynamic-reconfigure-param "/HSI_color_filter/hsi_filter" "h_limit_min" :int 60)
	   (ros::set-dynamic-reconfigure-param "/HSI_color_filter/hsi_filter" "s_limit_max" :int 255)
	   (ros::set-dynamic-reconfigure-param "/HSI_color_filter/hsi_filter" "s_limit_min" :int 90)
	   (ros::set-dynamic-reconfigure-param "/HSI_color_filter/hsi_filter" "i_limit_max" :int 255)
	   (ros::set-dynamic-reconfigure-param "/HSI_color_filter/hsi_filter" "i_limit_min" :int 70)))
	 )))


;; Look at table
(defun look-table ()
  (send *hironxjsk* :reset-manip-pose)
  (send *hironxjsk* :head :look-at
        (send (send (send *hironxjsk* :torso :end-coords :copy-worldcoords)
                    :translate #f(750 0 0)) :worldpos))
  (send *irtviewer* :draw-objects)  ;; Only for display
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1000)
  (send *ri* :wait-interpolation))


;;Before catch pose
(defun before-catch ()
  ;;to avoid to bump into a table
  (send *hironxjsk* :rarm_joint1 :joint-angle -50)
  (send-robot)
  (send-robot)
  (setq offset #f(0 0 200))
  (format t "cube pos ~A~%" (send *target-cube* :worldpos))
  (send *hironxjsk* :rarm 
	:inverse-kinematics (make-coords :pos (v+ (send *target-cube* :worldpos) offset)))
  (send-robot))

(defun catch-box ()
  (open-hand)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 -180) :world)
  (send-robot)
  (close-hand)
  ;;without this, fail to catch
  (ros::duration-sleep 2.0)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 200) :world)
  (send-robot))


(defun place-box (*target-place*)
  (send *hironxjsk* :rarm
	:inverse-kinematics (make-coords :pos *target-place*))
  (send-robot)
  (open-hand)
  (ros::duration-sleep 2.0)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 200))
  (send-robot)
  (send *hironxjsk* :reset-manip-pose)
  (send-robot))

;;Left-hand-version
(defun before-catch-l ()
  (send *hironxjsk* :larm_joint1 :joint-angle -50)
  (send-robot)
  (send-robot)
  (setq offset #f(0 0 200))
  (format t "cube pos ~A~%" (send *target-cube* :worldpos))
  (send *hironxjsk* :larm 
	:inverse-kinematics (make-coords :pos (v+ (send *target-cube* :worldpos) offset)))
  (send-robot))

(defun catch-box-l ()
  (open-hand-l)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 -180) :world)
  (send-robot)
  (close-hand-l)
  ;;without this, fail to catch
  (ros::duration-sleep 2.0)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 200) :world)
  (send-robot))

(defun place-box-l (*target-place*)
  (send *hironxjsk* :larm
	:inverse-kinematics (make-coords :pos *target-place*))
  (send-robot)
  (open-hand-l)
  (ros::duration-sleep 2.0)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 200))
  (send-robot)
  (send *hironxjsk* :chest_joint0 :joint-angle -90)
  (send-robot)
  (send *hironxjsk* :reset-manip-pose)
  (send *hironxjsk* :chest_joint0 :joint-angle -90)
  (send-robot)
  (send *hironxjsk* :reset-manip-pose)
  (send-robot))


;;Pick&Place Blue Box
(defun catch-blue ()
  (color-set 1)
  (look-table)
  (get-box-coords)
  (before-catch)
  (catch-box)
  (place-box #f(510 50 50)))


;;Pick&Place Green Box
(defun catch-green ()
  (color-set 2)
  (look-table)
  (get-box-coords)
  (before-catch)
  (catch-box)
  (place-box #f(510 -40 50)))
  

;;Pick&Place Red Box
(defun catch-red ()
  (setq find-box 0)
  (color-set 0)
  (look-table)
  ;;(ros::subscribe *bounding-box-list* jsk_recognition_msgs::BoundingBoxArray #'get-box-cb 1)
  (while (= find-box 0)
    (get-box-coords)
    (format t "failed, try again!~%"))
  (send *target-cube* :set-color :red)
  (objects *target-cube*) ;;For view
  (format t "find box")
  (before-catch-l)
  (catch-box-l)
  (place-box-l #f(550 0 130)))

(defun main ()
  (catch-blue)
  (catch-green)
  (catch-red))
  
;;memo #f(530 15 50)

;;失敗したやつ
;;If fail, try again
;;(defun get-box-cb (msg)
  ;;(setq *target-bbox* (elt (send msg :boxes) 0))
  ;;Get the shape and coords of the box
  ;;(setq *target-cube* (bbox->cube *target-bbox*))
  ;;(if (and (*box-x* < 65) (< *box-z* 45))
    ;;  (setq find-box 1))
  ;;(objects *target-cube*) ;;For view
  ;;)
