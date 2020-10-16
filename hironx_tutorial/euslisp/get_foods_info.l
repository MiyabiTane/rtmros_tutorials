#!/usr/bin/env roseus

(defun send-robot ()
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

(defun send-right-hand (sleep-time)
  (send *ri* :hand-angle-vector :rhand (send *hironxjsk* :hand-angle-vector :rhand) 2000)
  (ros::duration-sleep sleep-time)
  )

(defun send-left-hand (sleep-time)
  (send *ri* :hand-angle-vector :lhand (send *hironxjsk* :hand-angle-vector :lhand) 2000)
  (ros::duration-sleep sleep-time)
  )


(defun init-pose-20201001 ()
  (send *hironxjsk* :reset-manip-pose)
  (send *hironxjsk* :larm_joint4 :joint-angle -35)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 100) :world)
  (send *hironxjsk* :rarm_joint1 :joint-angle 20) 
  (send *hironxjsk* :rarm :move-end-pos #f(150 0 300))
  (send-robot)
  ;;look table
  (send *hironxjsk* :head :look-at
        (send (send (send *hironxjsk* :torso :end-coords :copy-worldcoords)
                    :translate #f(750 0 0)) :worldpos))
  (send *hironxjsk* :head :look-at #f(600 0 0))
  (send-robot)
  ;;open hand
  (send *hironxjsk* :hand-angle-vector :lhand #f(20.449 -20.449 -20.449 20.449))
  (send-left-hand 2.0)
  (send *hironxjsk* :hand-angle-vector :rhand #f(20.449 -20.449 -20.449 20.449))
  (send-right-hand 2.0)
  )


;;Only for draw
(defun draw (*food-pos* *box-len* *box-wid* *box-hei*)
  (setq *target-food* (make-cube *box-wid* *box-len* *box-hei*))
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
	  (setq coords-x (elt coords 0))
	  (setq coords-y (elt coords 1))
	  (setq food-pos-x (elt *food-pos* 0))
	  (setq food-pos-y (elt *food-pos* 1))
	  (setq dis (sqrt (+ (expt (- coords-x food-pos-x) 2) (expt (- coords-y food-pos-y) 2))))
	  ;;(setq dis (abs (- (norm coords) (norm *food-pos*))))
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

(init-pose-20201001)
(ros::roseus "screenpoint")
(ros::advertise *pub-topic* geometry_msgs::PointStamped 1)
(ros::subscribe *sub-topic* geometry_msgs::PointStamped #'screen_point_cb)
(coral_cb)

