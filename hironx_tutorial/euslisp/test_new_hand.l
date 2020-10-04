(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(hironxjsk-init)
(objects (list *hironxjsk*))

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

(defun new-init-pose ()
  ;;be careful! desk interferes with hiro's chopsticks!
  (send *hironxjsk* :reset-manip-pose)
  (send *hironxjsk* :larm :move-end-pos #f(100 0 300))
  (send *hironxjsk* :rarm_joint1 :joint-angle 20) 
  (send *hironxjsk* :rarm :move-end-pos #f(150 0 300))
  (send-robot)
  ;;look table
  (send *hironxjsk* :head :look-at
        (send (send (send *hironxjsk* :torso :end-coords :copy-worldcoords)
                    :translate #f(500 0 0)) :worldpos))
  (send-robot)
  ;;open hand
  (send *hironxjsk* :hand-angle-vector :lhand #f(20.449 -20.449 -20.449 20.449))
  (send-left-hand 2.0)
  (send *hironxjsk* :hand-angle-vector :rhand #f(20.449 -20.449 -20.449 20.449))
  (send-right-hand 2.0)
  )

;;diag-chopsticks
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


(defun open-chopsticks ()
  (send *hironxjsk* :hand-angle-vector :lhand #f(20.449 -20.449 -20.449 20.449))
  (send-left-hand 2.0)
  )

(defun open-tongs ()
  (send *hironxjsk* :hand-angle-vector :rhand #f(20.449 -20.449 -20.449 20.449))
  (send-right-hand 2.0)
  )

;;option 0 -> marshmallow, tomato(failed)
;;option 1 -> mini_tomato(failed), mugi_choco
(defun use-chopsticks (option)
  (new-init-pose)
  ;;move to food-pos
  (send *hironxjsk* :larm :move-end-pos #f(80 -75 0))
  (send-robot)

  ;;add for short chopsticks
  ;;(send *hironxjsk* :larm :move-end-pos #f(0 0 -145))
  ;;(send-robot)
  
  (send *hironxjsk* :larm :move-end-pos #f(0 0 -75))
  (send-robot)
  ;;close-chopsticks
  (if (= option 0)
      (send *hironxjsk* :hand-angle-vector :lhand #f(-10.449 10.449 10.449 -10.449)))
  (if (= option 1)
      (send *hironxjsk* :hand-angle-vector :lhand #f(-15.449 15.449 15.449 -15.449)))
  (send-left-hand 2.0)
  ;;move to release-pos
  (send *hironxjsk* :larm :move-end-pos #f(0 -150 50))
  (send-robot)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 -50))
  (send-robot)
  ;;open-chopstics
  (open-chopsticks)
  ;;new-init-pose
  (send *hironxjsk* :larm :move-end-pos #f(0 0 50))
  (send-robot)
  (new-init-pose)
  )

;;option 0 -> felt_rolled_egg
;;option 1 -> rolled_egg, mini_tomato
(defun use-tongs (option)
  (new-init-pose)
  ;;move to food-pos
  (send *hironxjsk* :rarm :move-end-pos #f(40 50 0))
  (send-robot)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 -90))
  (send-robot)
  ;;close tongs
  (when (= option 0)
    (send *hironxjsk* :hand-angle-vector :rhand #f(-10.449 10.449 10.449 -10.449))
    (send-right-hand 2.0)
    (send *hironxjsk* :hand-angle-vector :rhand #f(-15.449 15.449 15.449 -15.449))
    (send-right-hand 0.0))
  (when (= option 1)
    (send *hironxjsk* :hand-angle-vector :rhand #f(-15.449 15.449 15.449 -15.449))
    (send-right-hand 2.0))
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 50))
  (send-robot)
  ;;move to release-pos
  (send *hironxjsk* :rarm :move-end-pos #f(0 150 0))
  (send-robot)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 -45))
  (send-robot)
  ;;open-tongs
  (open-tongs)
  ;;new-init-pose
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 50))
  (send-robot)
  (new-init-pose)
  )

(defun stuff_experiment ()
  (new-init-pose)
  ;;move to food-pos
  (send *hironxjsk* :rarm :move-end-pos #f(40 50 0))
  (send-robot)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 -90))
  (send-robot)
  ;;close tongs
  (send *hironxjsk* :hand-angle-vector :rhand #f(-15.449 15.449 15.449 -15.449))
  (send-right-hand 2.0)
  ;;move to release-pos
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 100))
  (send-robot)
  (send *hironxjsk* :rarm :move-end-pos #f(-50 120 0))
  (send-robot)
  (send *hironxjsk* :rarm :move-end-pos #f(20 0 -70))
  (send-robot)
  ;;open one tong
  (send *hironxjsk* :hand-angle-vector :rhand #f(-15.449 15.449 -20.449 20.449))
  (send-right-hand 2.0)
  ;;new-init-pose
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 70))
  (send-robot)
  (new-init-pose)
  ;;use-chopsticks(short)
  (send *hironxjsk* :hand-angle-vector :lhand #f(-10.449 10.449 10.449 -10.449))
  (send-left-hand 2.0)
  ;;memo
  ;;tomato no iti made ugokasu
  (send *hironxjsk* :larm :move-end-pos #f(40 -150 0))
  (send-robot)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 -170))
  (send-robot)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 -30))
  (send-robot)
  (send *hironxjsk* :larm :move-end-pos #f(-10 0 0))
  (send-robot)
  (send *hironxjsk* :larm :move-end-pos #f(20 0 100))
  (send-robot)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 -100))
  (send-robot)
  (send *hironxjsk* :larm :move-end-pos #f(10 0 0))
  (send-robot)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 100))
  (send-robot)
  (new-init-pose)
  )
