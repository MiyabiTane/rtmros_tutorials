;;Rostopic
(ros::load-ros-package "jsk_recognition_msgs")

(defclass edgetpu_boxes_synchronizer
  :super exact-time-message-filter)
(defmethod edgetpu_boxes_synchronizer
  (:callback (class boxes)
    ;; do something
    (print class)
    (print boxes)
    )
  )

;; Look at table
(defun look-table ()
  (send *hironxjsk* :reset-manip-pose)
  (send *hironxjsk* :head :look-at
        (send (send (send *hironxjsk* :torso :end-coords :copy-worldcoords)
                    :translate #f(750 0 0)) :worldpos))
  (send *irtviewer* :draw-objects)  ;; Only for display
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  )


(look-table) ;;for simulation
;;(init-pose-20201001)
(let (sync)
  (setq sync
        (instance edgetpu_boxes_synchronizer :init
                  (list (list "/edgetpu_object_detector/output/class" jsk_recognition_msgs::ClassificationResult)
                        (list "/segmentation_decomposer_ssd/boxes" jsk_recognition_msgs::BoundingBoxArray))))
  (do-until-key (ros::spin-once) (ros::duration-sleep 0.01)))

(ros::unsubscribe "/edgetpu_object_detector/output/class")
(ros::unsubscribe "/segmentation_decomposer_ssd/boxes")
