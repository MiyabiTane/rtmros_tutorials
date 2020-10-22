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

;;同じtimeStampedの情報を取ってくる
;;親クラスが呼ばれた時、Subscribeされる
(defclass edgetpu_boxes_synchronizer
  :super exact-time-message-filter)
(defmethod edgetpu_boxes_synchronizer
  (:callback (class boxes)
	     (print class)
	     (print boxes)
	     (when (> length name_list 4) 
	       (setq name_list (send class :label_names))
	       (format t "name_list ~A~%" name_list)
	       (dotimes (i (length (send boxes :boxes)))
		 (setq bbox (elt boxes i))
		 (setq cascoords (send (send *tfl* :lookup-transform "WAIST" (send bbox :header :frame_id) (ros::time 0))
				       :transform (ros::tf-pose->coords (send bbox :pose))))
		 (setq coords (send cascoords :worldpos))
		 (format t "coords ~A~%" coords)
		 (setq pos_list (list-insert coords (length pos_list) pos_list))
		 (setq size (ros::tf-point->pos (send bbox :dimensions)))
		 (setq size_list (list (elt size 0) (elt size 1) (elt size 2)))
		 (format t "size ~A~%" size_list)
		 (setq box_list (list-insert size_list (length box_list) box_list))
		 )
	       (setq *fin-f* t)
	       (ros::unsubscribe "/edgetpu_object_detector/output/class")
	       (ros::unsubscribe "/segmentation_decomposer_ssd/boxes")
	       ))
  )

;;(look-table) ;;for simulation
;;(init-pose-20201001)
(move_hand_out_of_camera)
(setq *fin-f* nil)
(let (sync)
  (setq sync
        (instance edgetpu_boxes_synchronizer :init
                  (list (list "/edgetpu_object_detector/output/class" jsk_recognition_msgs::ClassificationResult)
                        (list "/segmentation_decomposer_ssd/boxes" jsk_recognition_msgs::BoundingBoxArray))))
  (until *fin-f*
	 (ros::spin-once) (ros::duration-sleep 0.01)
	 )
  )

;;(ros::unsubscribe "/edgetpu_object_detector/output/class")
;;(ros::unsubscribe "/segmentation_decomposer_ssd/boxes")
