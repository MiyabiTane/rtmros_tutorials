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
  )

(defun open-chopsticks ()
  (send *hironxjsk* :hand-angle-vector :lhand #f(20.449 -20.449 -20.449 20.449))
  (send-left-hand 2.0)
  )

(defun marshmallow-chopsticks () ;;succeeded
  ;;open-chopsticks
  (open-chopsticks)
  ;;move to food-pos
  (send *hironxjsk* :larm :move-end-pos #f(80 -75 0))
  (send-robot)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 -75))
  (send-robot)
  ;;close-chopsticks(marshmallow)
  (send *hironxjsk* :hand-angle-vector :lhand #f(-10.449 10.449 10.449 -10.449))
  (send-left-hand 2.0)
  ;;move to release-pos
  (send *hironxjsk* :larm :move-end-pos #f(0 -150 20))
  (send-robot)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 -20))
  (send-robot)
  ;;open-chopstics
  (open-chopsticks)
  ;;new-init-pose
  (send *hironxjsk* :larm :move-end-pos #f(0 0 50))
  (send-robot)
  (new-init-pose)
  )
  
(defun rolled_egg-chopsticks () ;;failed because rolled_egg is heavy
  ;;open-chopsticks
  (open-chopsticks)
  ;;move to food-pos
  (send *hironxjsk* :larm :move-end-pos #f(80 -75 0))
  (send-robot)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 -75))
  (send-robot)
  ;;close-chopsticks
  (send *hironxjsk* :hand-angle-vector :lhand #f(-15.449 15.449 15.449 -15.449))
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

(defun mugi_choco-chopsticks () ;;succeeded
  ;;open-chopsticks
  (open-chopsticks)
  ;;move to food-pos
  (send *hironxjsk* :larm :move-end-pos #f(80 -75 0))
  (send-robot)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 -75))
  (send-robot)
  ;;close-chopsticks
  (send *hironxjsk* :hand-angle-vector :lhand #f(-15.449 15.449 15.449 -15.449))
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

(defun mini_tomato-chopsticks () ;;failed because tsurutsuru
  ;;open-chopsticks
  (open-chopsticks)
  ;;move to food-pos
  (send *hironxjsk* :larm :move-end-pos #f(80 -75 0))
  (send-robot)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 -75))
  (send-robot)
  ;;close-chopsticks
  (send *hironxjsk* :hand-angle-vector :lhand #f(-10.449 10.449 10.449 -10.449))
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
