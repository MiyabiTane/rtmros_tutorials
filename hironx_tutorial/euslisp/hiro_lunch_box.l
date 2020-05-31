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
(defvar *bounding-box-list* "/segmentation_decomposer/boxes")
;;(defvar *bounding-box-list* "/HSI_color_filter/boxes")


;;keep info
(setq name_list '())
(setq pos_list '())
(setq box_list '())


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
(defun draw (*food-pos* *box-len* *box-wid* *box-hei*)
  (setq *target-food* (make-cube *box-len* *box-wid* *box-hei*))
  (send *target-food* :locate *food-pos*)
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
(defun coral_cb ()
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
      (setq *leng* (length (send *food-name-msg* :label_names)))
      (format t "list length ~A~%" *leng*)
      (format t "got data from coral~%")
      (setq index 0)
      (dolist (str (send *food-name-msg* :label_names))
	(setq catch-pubinfo 0)
	(format t "find ~A~%" str)
	(setq name_list (append name_list (list str)))
	(setq *food-rects* (elt (send *food-rects-msg* :rects) index))
	(setq index (+ index 1))
	(publish-center-coords *food-rects*)
	(until (= catch-pubinfo 1)
	       (ros::spin-once)
	       (ros::sleep)
	       ))
      (format t "name_list is ~A~%" name_list)
      ))
  )


(defun screenpoint->world (msg)
  ;;ros::tf-point->coords *camera-frame*座標を"WAIST"座標に変換
  (let* ((waist-to-camera (send *tfl* :lookup-transform "WAIST" *camera-frame* (ros::time 0)))
	 (coords (make-coords :pos (ros::tf-point->pos (send msg :point)))))
    (send coords :transform waist-to-camera :world)
    (format t "food coords ~A~%" (send coords :pos))
    (setq *food-pos* (send coords :pos))
    *food-pos*))


;;screen-point CallBack
(defun screen_point_cb (msg)
  (format t "screen_point_cb~%")
  (setq *food-pos* (screenpoint->world msg))
  (setq pos_list (list-insert *food-pos* (length pos_list) pos_list))
  (setq catch-pubinfo 1)
  (when (= (length pos_list) *leng*)
    (format t "pos_list is ~A~%" pos_list)
    (format t "get box~%")
    (ros::unsubscribe *sub-topic*)
    (get_box_info pos_list))
  )


;;bounding-box
(defun get_box_info (pos_list)
  (dotimes (i (length pos_list))
    (setq *food-pos* (elt pos_list i))
    (format t "*food-pos* ~A~%" *food-pos*)
    (setq *box-len* nil)
    (while (eql *box-len* nil)
      ;;(format t "subscribe~%")
      (setq *bbox-msg* (one-shot-subscribe *bounding-box-list*
					   jsk_recognition_msgs::BoundingBoxArray
					   :after-stamp (ros::time-now)))
      (when (> (length (send *bbox-msg* :boxes)) 0)
	(format t "get bbox-list~%")
	(setq min_dis 1000000000000)
	;;求めたおかずの位置に最も近いboxを取ってくる
	(dotimes (i (length (send *bbox-msg* :boxes)))
	  (setq bbox (elt (send *bbox-msg* :boxes) i))
	  (setq cascoords (send (send *tfl* :lookup-transform "WAIST" (send bbox :header :frame_id) (ros::time 0))
				:transform (ros::tf-pose->coords (send bbox :pose))))
	  ;;(format t "cascoords is ~A~%" cascoords)
	  (setq coords (send cascoords :worldpos))
	  (format t "coords is ~A~%" coords)
	  (setq dis (abs (- (norm coords) (norm *food-pos*))))
	  (if (< dis min_dis)
	      (progn
		(format t "distance smaller~%")
		(setq min_dis dis)
		(setq size (ros::tf-point->pos (send bbox :dimensions)))
		(setq *box-len* (elt size 0))
		(setq *box-wid* (elt size 1))
		(setq *box-hei* (elt size 2))
		))))
      (ros::spin-once)
      (ros::sleep)
      )
    (format t "box size ~A ~A ~A~%" *box-len* *box-wid* *box-hei*)
    (draw *food-pos* *box-len* *box-wid* *box-hei*)
    (setq sub_list (list *box-len* *box-wid* *box-hei*))
    (setq box_list (list-insert sub_list (length box_list) box_list))
    )
  (format t "box_list is ~A~%" box_list)
  ;;(ros::exit)
  )

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

(look-table)
(ros::roseus "screenpoint")
(ros::advertise *pub-topic* geometry_msgs::PointStamped 1)
(ros::subscribe *sub-topic* geometry_msgs::PointStamped #'screen_point_cb)
(coral_cb)

(format t "name_list ~A~%" name_list)
(format t "pos_list ~A~%" pos_list)
(format t "box_list ~A~%" box_list)

(find_best_stuff name_list pos_list box_list)
(stuff-all name_list pos_list place_list)




  
  
  
  
