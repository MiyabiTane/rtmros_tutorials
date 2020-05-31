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

(defvar *attention-box-list* "/attention_clipper/output/box_array")


;; Look at table
(defun look-table ()
  (send *hironxjsk* :reset-manip-pose)
  (send *hironxjsk* :head :look-at
        (send (send (send *hironxjsk* :torso :end-coords :copy-worldcoords)
                    :translate #f(750 0 0)) :worldpos))
  (send *irtviewer* :draw-objects)  ;; Only for display
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1000)
  (send *ri* :wait-interpolation))


(defun tf-pose->coords (frame_id pose)
  (send (send *tfl* :lookup-transform "WAIST" frame_id (ros::time 0))
        :transform (ros::tf-pose->coords pose)))


(defun bbox->cube (bbox)
  (let* ((dims (ros::tf-point->pos (send bbox :dimensions)))
         (bx (make-cube (elt dims 0) (elt dims 1) (elt dims 2))))
    (send bx :newcoords
          (tf-pose->coords (send bbox :header :frame_id) (send bbox :pose)))
    bx))


(defun attention-cb ()
  (setq find-box 0)
  (while (eql find-box 0)
    (setq *atbox-msg* (one-shot-subscribe *attention-box-list*
					  jsk_recognition_msgs::BoundingBoxArray
					  :after-stamp (ros::time-now)))
    (when (> (length (send *atbox-msg* :boxes)) 0)
      (setq *target-atbox* (elt (send *atbox-msg* :boxes) 0))
      (setq *target-cube* (bbox->cube *target-atbox*))
      (objects *target-cube*)
      (setq find-box 1))
    ))


(look-table)
;;(attention-cb)
(send *hironxjsk* :head :look-at #f(688 -237 16))
(send *ri* :angle-vector (send *hironxjsk* :angle-vector) 5000)
(send *ri* :wait-interpolation)
;;(attention-cb)

;;(while (ros::ok)
  ;;(ros::spin-once)
  ;;(ros::sleep)
  ;;)
