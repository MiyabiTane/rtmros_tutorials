#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(hironxjsk-init)
(objects (list *hironxjsk*))

(defun set-angle ()
  (send *hironxjsk* :rarm_joint5 :joint-angle 70)
  (send *hironxjsk* :larm_joint5 :joint-angle 70)
  )

(defun send-robot ()
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

(defun send-rhand (sleep-time)
  (send *ri* :hand-angle-vector :rhand (send *hironxjsk* :hand-angle-vector :rhand) 2000)
  (ros::duration-sleep sleep-time)
  )

(defun send-lhand (sleep-time)
  (send *ri* :hand-angle-vector :lhand (send *hironxjsk* :hand-angle-vector :lhand) 2000)
  (ros::duration-sleep sleep-time)
  )

(defun open-lhand ()
  (send *hironxjsk* :hand-angle-vector :lhand #f(10.449 -10.449 -10.449 10.449))
  (send-lhand 2.0))

(defun open-rhand ()
  (send *hironxjsk* :hand-angle-vector :rhand #f(10.449 -10.449 -10.449 10.449))
  (send-rhand 2.0))

(defun close-lhand ()
  (send *ri* :hand-width :lhand 0))

(defun close-rhand ()
  (send *ri* :hand-width :rhand 0))

(defun look-73b2-table ()
  ;;look table
  (send *hironxjsk* :head :look-at
        (send (send (send *hironxjsk* :torso :end-coords :copy-worldcoords)
                    :translate #f(750 0 0)) :worldpos))
  (send *hironxjsk* :head :look-at #f(500 0 0))
  ;;(send-robot)
  )

(defun rotate-rhand (degree)
  (setq radi (deg2rad degree))
  (setq *hiro-coords* (send (send (send *hironxjsk* :rarm :end-coords) :copy-worldcoords) :pos))
  (setq *hiro-coords* (make-cascoords :pos *hiro-coords*))
  (send (send *hironxjsk* :rarm :end-coords) :assoc *hiro-coords*)
  (setq new-coords (send *hiro-coords* :copy-worldcoords))
  (send new-coords :rotate radi :y :world)
  (send *hironxjsk* :rarm :inverse-kinematics new-coords :move-target *hiro-coords*)
  )

(defun rotate-lhand (degree)
  (setq radi (deg2rad degree))
  (setq *hiro-coords* (send (send (send *hironxjsk* :larm :end-coords) :copy-worldcoords) :pos))
  (setq *hiro-coords* (make-cascoords :pos *hiro-coords*))
  (send (send *hironxjsk* :larm :end-coords) :assoc *hiro-coords*)
  (setq new-coords (send *hiro-coords* :copy-worldcoords))
  (send new-coords :rotate radi :y :world)
  (send *hironxjsk* :larm :inverse-kinematics new-coords :move-target *hiro-coords*)
  )

(defun move_rarm (pos)
  (send *hironxjsk* :rarm :move-end-pos pos :world)
  (send-robot)
  )

(defun move_larm (pos)
  (send *hironxjsk* :larm :move-end-pos pos :world)
  (send-robot)
  )

(defun init-pose ()
  (send *hironxjsk* :reset-manip-pose)
  (send *hironxjsk* :larm :move-end-pos #f(100 0 200) :world)
  (send *hironxjsk* :rarm :move-end-pos #f(100 0 200) :world)
  (look-73b2-table)
  (send-robot)
  (open-rhand)
  (open-lhand)
  )

(defun main ()
  (move_rarm #f(100 0 100))
  (move_larm #f(100 0 100))
  (move_larm #f(-100 -100 -100))
  (move_rarm #f(0 100 0))
  ;;持ち替え
  (setq cur_lcoords (send (send (send *hironxjsk* :larm :end-coords) :copy-worldcoords) :pos))
  ;;(open-rhand)
  (move_rarm #f(0 0 50))
  ;;ぶつかるので左手を避ける
  ;;(open-lhand)
  (move_larm #f(-100 0 0))

  (send *hironxjsk* :rarm :inverse-kinematics (make-coords :pos (v+ cur_lcoords #f(50 0 0))))
  (send-robot)
  ;;(move_rarm #f(0 0 -50))
  ;;(close-rhand)

  )
