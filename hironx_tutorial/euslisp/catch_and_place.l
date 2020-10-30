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

(defun open-tongs ()
  (send *hironxjsk* :hand-angle-vector :rhand #f(15.449 -15.449 -15.449 15.449))
  (send-right-hand 2.0)
  )

(defun close-tongs ()
  (send *hironxjsk* :hand-angle-vector :rhand #f(-15.499 15.499 15.499 -15.499))
  (send-right-hand 2.0)
  )	

(defun catch-food (pos)
  ;;same pose 
  (setq *hiro-pose* (send (send *hironxjsk* :rarm :end-coords) :copy-worldcoords))
  (setq pos (v- pos #f(-22.364 164.268 218.072))) ;;tong offset
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
  ;;same pose
  (setq *hiro-pose* (send (send *hironxjsk* :rarm :end-coords) :copy-worldcoords))
  (setq pos (v- pos #f(-22.364 164.268 218.072))) ;;tong offset
  (setq pos (v+ pos *coords*)) ;;*coords*<-lunchbox pos
  (setq pos (v+ pos (float-vector 0 0 (- *catch-z* (elt pos 2))))) ;;set z *catch-z*
  (setq *move* (v- pos (send *hiro-pose* :pos)))
  (send *hiro-pose* :translate *move* :world)
  (send *hironxjsk* :rarm :inverse-kinematics *hiro-pose*)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 100))
  (send-robot)
  (open-tongs)
  )

(defun main()
  ;;(setq pos_list (list #f(356.094 -155.46 -78.2884) #f(392.641 -112.817 -75.8539)))
  ;;(setq place_list (list #f(0.0 0.0 28.1427) #f(0.0 0.0 19.4847)))
  (init-pose-20201030)
  (dotimes (i (length place_list))
    (when (> (elt (elt place_list i) 0) 0)
      (catch-food (elt pos_list i))
      (place-food (elt place_list i))
      )
    )
  )

(main)

;;#f(-110.25 9.0407 64.6251) hand-pos - real-pos(rarm)
;;tong(rotate 90)
;;#f(426.375 22.469 -46.8001) real
;;#f(404.011 186.737 171.272) hand
;;#f(-22.364 164.268 218.072) ;;hand-pos - real-pos
