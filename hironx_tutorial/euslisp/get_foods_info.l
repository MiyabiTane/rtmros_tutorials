#!/usr/bin/env roseus

;;Rostopic
(ros::load-ros-package "jsk_recognition_msgs")
(setq *camera-frame* "head_camera_rgb_optical_frame")


;;keep info
(setq name_list '())
(setq pos_list '())
(setq box_list '())

;;use tf
(unless (boundp '*tfl*)
  (setq *tfl* (instance ros::transform-listener :init)))


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

;;move hand out of camera view
(defun move_hand_out_of_camera ()
  (init-pose-20201001)
  (send *hironxjsk* :larm :move-end-pos #f(0 200 0))
  (send-robot)
  (send *hironxjsk* :rarm :move-end-pos #f(0 -200 0))
  (send-robot)
  (send *hironxjsk* :rarm_joint5 :joint-angle 0)
  (send-robot)
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

;;ほぼ同じtimeStampedの情報を取ってくる
(defun get_box_cb (msg)
  (setq b_cb_lst (append b_cb_lst (list msg)))
  (format t "Called box~%")
  ;;(format t "box list ~A~%" b_cb_lst)
  )

(defun get_name_cb (msg)
  (setq n_cb_lst (append n_cb_lst (list msg)))
  ;;(format t "name list ~A~%" n_cb_lst)
  (format t "Called name~%")
  )
      
(defun sync_info ()
  (dotimes (i 2)
    (setq b_time (send (send (send (elt b_cb_lst i) :header) :stamp) :sec-nsec))
    (format t "time ~A~%" b_time)
    (dotimes (j 2)
      (setq c_time (send (send (send (elt n_cb_lst j) :header) :stamp) :sec-nsec))
      (format t "time ~A~%" c_time)
      (when (and (= (elt b_time 0) (elt c_time 0)) (< (abs (- (elt b_time 1) (elt c_time 1))) 10000000))
	(when (= (length name_list) 0)
	  (setq class (elt n_cb_lst j))
	  (setq name_list (send class :label_names))
	  ;;(format t "name_list ~A~%" name_list)
	  (setq bbox-msg (elt b_cb_lst i))
	  (setq boxes (send bbox-msg :boxes))
	  (dotimes (k (length boxes))
	    (setq bbox (elt boxes k))
	    (setq cascoords (send (send *tfl* :lookup-transform "WAIST" (send bbox :header :frame_id) (ros::time 0))
				  :transform (ros::tf-pose->coords (send bbox :pose))))
	    (setq coords (send cascoords :worldpos))
	    ;;(format t "coords ~A~%" coords)
	    (setq pos_list (list-insert coords (length pos_list) pos_list))
	    (setq size (ros::tf-point->pos (send bbox :dimensions)))
	    (setq size_list (list (elt size 0) (elt size 1) (elt size 2)))
	    ;;(format t "size ~A~%" size_list)
	    (setq box_list (list-insert size_list (length box_list) box_list))
	    )
	  (ros::unsubscribe "/edgetpu_boxes_synchronizer/pub_00")
	  (ros::unsubscribe "/edgetpu_boxes_synchronizer/pub_01")
	  ))))
  ;;Retry
  ;;(setq b_cb_lst '())
  ;;(setq n_cb_lst '())
  ;;(main_loop)
  )

(defun main_loop ()
  (while (and (< (length b_cb_lst) 2) (< (length n_cb_lst) 2))
    ;;(format t "Called~%")
    (ros::spin-once)
    (ros::sleep)
    )
  (sync_info)
  )

;;(look-table) ;;for simulation
;;(init-pose-20201001)
(move_hand_out_of_camera)
(setq b_cb_lst '())
(setq n_cb_lst '())

(ros::subscribe "/edgetpu_boxes_synchronizer/pub_00" jsk_recognition_msgs::ClassificationResult #'get_name_cb)
(ros::subscribe "/edgetpu_boxes_synchronizer/pub_01" jsk_recognition_msgs::BoundingBoxArray #'get_box_cb)
(main_loop)
