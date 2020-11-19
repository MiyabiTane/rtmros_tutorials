#!/usr/bin/env roseus

(defun open-tongs ()
  (send *hironxjsk* :hand-angle-vector :rhand #f(15.449 -15.449 -15.449 15.449))
  (send-right-hand 2.0)
  )

(defun close-tongs ()
  ;;(send *hironxjsk* :hand-angle-vector :rhand #f(-15.499 15.499 15.499 -15.499))
  (send *hironxjsk* :hand-angle-vector :rhand #f(-20.499 20.499 20.499 -20.499))
  (send-right-hand 2.0)
  )	

(defun catch-food (pos)
  ;;same pose 
  (setq *hiro-pose* (send (send *hironxjsk* :rarm :end-coords) :copy-worldcoords))
  (setq pos (v+ pos *tong-offset*)) ;;tong offset
  (setq pos (v+ pos (float-vector 0 0 (- *catch-z* (elt pos 2))))) ;;set z *catch-z*
  (setq *move* (v- pos (send *hiro-pose* :pos)))
  (send *hiro-pose* :translate *move* :world)
  (send *hironxjsk* :rarm :inverse-kinematics *hiro-pose*)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 70) :world)
  (send-robot)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 -70) :world)
  (send-robot)
  (close-tongs)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 70) :world)
  (send-robot)
  )

(defun place-food (pos)
  ;;(format t "pos: ~A~%" pos)
  ;;same pose
  (setq *hiro-pose* (send (send *hironxjsk* :rarm :end-coords) :copy-worldcoords))
  (setq pos (float-vector (elt pos 0) (* (elt pos 1) -1) (elt pos 2))) 
  (setq pos (v+ pos (float-vector *front* *left* 0))) ;;*coords*<-lunchbox pos
  ;;(format t "target food pos: ~A~%" pos)
  (setq pos (v+ pos *tong-offset-2*)) ;;tong offset
  (setq pos (v+ pos (float-vector 0 0 (- *catch-z* (elt pos 2))))) ;;set z *catch-z*
  (setq *move* (v- pos (send *hiro-pose* :pos)))
  (send *hiro-pose* :translate *move* :world)
  ;;(format t "hiro_pose ~A~%" *hiro-pose*)
  (send *hironxjsk* :rarm :inverse-kinematics *hiro-pose*)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 100) :world)
  (send-robot)
  (open-tongs)
  )

(setq *tong-offset* #f(-39.159 -114.508 100.161))
(setq *tong-offset-2* #f(0 -114.508 100.161))
(defun catch_and_place_main()
  (format t "- - - CATCH AND PLACE - - -~%")
  ;;(setq pos_list (list #f(356.094 -155.46 -78.2884) #f(392.641 -112.817 -75.8539)))
  ;;(setq place_list (list #f(0.0 0.0 28.1427) #f(0.0 0.0 19.4847)))
  (init-pose-20201030)
  (open-tongs)
  (dotimes (i (length place_list))
    (when (> (elt (elt place_list i) 0) 0)
      (catch-food (elt pos_list i))
      (place-food (elt place_list i))
      )
    )
  )

;;#f(-110.25 9.0407 64.6251) hand-pos - real-pos(rarm)
;;tong(rotate 90)
;;#f(-39.159 -114.508 100.161)
