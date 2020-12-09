#!/usr/bin/env roseus

(ros::advertise "/before_place_img" sensor_msgs::Image 1)
(ros::advertise "/placed_img" sensor_msgs::Image 1)

(defun place-init-pose ()
  (send *hironxjsk* :reset-manip-pose)
  (send *hironxjsk* :larm_joint4 :joint-angle -35)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 100) :world)
  (send *hironxjsk* :rarm :move-end-pos #f(50 0 150) :world)
  (send *hironxjsk* :rarm_joint5 :joint-angle -75)
  (send *hironxjsk* :rarm_joint3 :joint-angle 35)
  (send *hironxjsk* :rarm_joint4 :joint-angle 48)
  (look-73b2-table)
  (send *hironxjsk* :larm :move-end-pos #f(0 160 0) :world)
  (send-robot)
  )

(defun open-chops (size)
  (setq size_zero #f(-30.449 30.449 30.449 -30.449))
  (setq size_23 #f(15.449 -15.449 -15.449 15.449))
  (setq plus (scale (/ size 23.0) size_23))
  (setq target_hand (v+ size_zero plus))
  (send *hironxjsk* :hand-angle-vector :lhand target_hand)
  (send-left-hand 2.0)
  )

(defun open-tongs (type)
  ;;type 0 -> small object
  ;;type 1 -> big object
  ;;type 2 -> トングの先が広がったポーズ
  (cond ((= type 0)
	 (send *hironxjsk* :hand-angle-vector :rhand #f(15.449 -15.449 -15.449 15.449)))
	((= type 1)
	 (send *hironxjsk* :hand-angle-vector :rhand #f(25.449 -25.449 -25.449 25.449)))
	((= type 2)
	 (send *hironxjsk* :hand-angle-vector :rhand #f(10 10 -10 -10))))
  (send-right-hand 2.0)
  )

(defun close-tongs ()
  (if (v= (send *hironxjsk* :hand-angle-vector :rhand) #f(10 10 -10 -10))
      (open-tongs 1))
  (setq close #f(-10 10 10 -10))
  (setq diff 0)
  (while (< diff 2.0)
    (setq prev_hand (send *ri* :hand-angle-vector :rhand))
    (send *hironxjsk* :hand-angle-vector :rhand (v+ prev_hand close))
    (send-right-hand 2.0)
    (setq cur_hand (send *ri* :hand-angle-vector :rhand))
    (setq diff (norm (v- (v- cur_hand prev_hand) close)))
    ;;(format t "diff ~A~%" diff)
    (when (< (elt (send *hironxjsk* :hand-angle-vector :rhand) 0) -16)
      (format t "enough~%")
      (setq diff 3))
    (send-right-hand 2.0)
    )
  (format t "diff ~A~%" diff)
  )

(defun rotate-tongs (radi)
  (format t "rotation ~A~%" (rad2deg radi))
  (setq *hiro-coords* (send (send (send *hironxjsk* :rarm :end-coords) :copy-worldcoords) :pos))
  (setq tong-coords (make-cascoords :pos (v- (v- *hiro-coords* *tong-offset*) #f(0 0 70))))
  (send (send *hironxjsk* :rarm :end-coords) :assoc tong-coords)
  (setq target-tong-coords (send tong-coords :copy-worldcoords))
  (send target-tong-coords :rotate radi :z :world)
  (send *hironxjsk* :rarm :inverse-kinematics target-tong-coords :move-target tong-coords)
  (send-robot)
  )

(defun de-rotate-tongs (radi)
  (setq target-tong-coords (send tong-coords :copy-worldcoords))
  (send target-tong-coords :rotate radi :z :world)
  (send *hironxjsk* :rarm :inverse-kinematics target-tong-coords :move-target tong-coords)
  (send-robot)
  )

(defun sub-img (num)
  (cond ((= num 0)
	 (setq *bimg-msg* (one-shot-subscribe "/head_camera/rgb/image_raw"
					      sensor_msgs::Image
					      :after-stamp (ros::time-now))))
	((= num 1)
	 (setq *aimg-msg* (one-shot-subscribe "/head_camera/rgb/image_raw"
					      sensor_msgs::Image
					      :after-stamp (ros::time-now)))))
  )
 
(defun catch-food (pos radi)
  ;;same pose 
  (setq *hiro-pose* (send (send *hironxjsk* :rarm :end-coords) :copy-worldcoords))
  (if (= radi 0)
      (setq pos (v+ pos (v+ *tong-offset* #f(10 0 0)))) ;;tong offset
    (setq pos (v+ pos (v+ *tong-offset* #f(0 14 0)))) ;;内側め
    )
  (setq pos (v+ pos (float-vector 0 0 (- *catch-z* (elt pos 2))))) ;;set z *catch-z*
  (setq *move* (v- pos (send *hiro-pose* :pos)))
  (send *hiro-pose* :translate *move* :world)
  (send *hironxjsk* :rarm :inverse-kinematics *hiro-pose*)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 70) :world)
  (send-robot)
  ;;意図的調節
  (cond ((> radi 5)
	 (setq radi (- radi (/ (* 5 pi) 180))))
	((< radi -5)
	 (setq radi (+ radi (/ (* 5 pi) 180)))))
  ;;おかずに合わせたトングの回転
  (rotate-tongs radi)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 -70) :world)
  (send-robot)
  (close-tongs)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 70) :world)
  (send-robot)
  ;;元の向きに戻す
  (send target-tong-coords :rotate (* -1 radi) :z :world)
  (send *hironxjsk* :rarm :inverse-kinematics target-tong-coords :move-target tong-coords)
  (send-robot)
  )

(defun pull_over (pos food-size lbox_pos diff)
  ;;おかずの大きさに合わせて開く
  (open-chops (+ (elt food-size 1) 10))
  ;;pos GAでの座標 x->x, y->-y
  ;;おかずはお弁当箱の真ん中に置かれる
  (setq goal_x (elt pos 0))
  (setq goal_y (elt pos 1))
  (setq center_x (/ *length* 2))
  (setq center_y (/ *width* 2))
  (setq size_x (elt food-size 0))
  (setq size_y (elt food-size 1))
  (format t "goal_x: ~A center_x: ~A~%" goal_x center_x)
  (format t "goal_y: ~A center_y: ~A~%" goal_y center_y)
  ;;置く時にずれた分
  (setq diff_x (* (/ (elt diff 1) lbox_length) *length*))
  (setq diff_y (* (/ (* -1 (elt diff 0)) lbox_length) *length*))
  (setq center_x (+ center_x diff_x))
  (setq center_y (+ center_y diff_y))
  (format t "diff_x diff_y ~A ~A~%" diff_x diff_y)
  ;;上下寄せ
  (setq *hiro-pose* (send (send *hironxjsk* :larm :end-coords) :copy-worldcoords))
  (setq start-pos (v+ lbox_pos (float-vector 0 0 (+ *G-z* 60))))
  (setq start-pos (v+ start-pos *chopstick-offset*))
  (setq *move* (v- start-pos (send *hiro-pose* :pos)))
  (send *hiro-pose* :translate *move* :world)
  (send *hironxjsk* :larm :inverse-kinematics *hiro-pose*)
  (send-robot)
  ;;ずれの分
  (send *hironxjsk* :larm :move-end-pos (float-vector diff_x diff_y 0) :world)
  (send-robot)
  ;;pull-over
  (send *hironxjsk* :larm :move-end-pos #f(0 0 -70) :world)
  (send-robot)
  (send *hironxjsk* :hand-angle-vector :lhand (v+ (send *hironxjsk* :hand-angle-vector :lhand) #f(-7 7 7 -7)))
  (send-left-hand 2.0)
  (send *hironxjsk* :larm :move-end-pos (float-vector (- goal_x center_x) 0 0) :world)
  (cond ((> (- goal_x center_x) 0)
	 (send *hironxjsk* :larm :move-end-pos #f(5 0 0) :world))
	((< (- goal_x center_x) 0)
	 (send *hironxjsk* :larm :move-end-pos #f(-5 0 0) :world)))
  (send-robot)
  (open-chops (+ (elt food-size 1) 10))
  (send *hironxjsk* :larm :move-end-pos #f(0 0 30) :world)
  (send-robot)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 -30) :world)
  (send-robot)
  (send *hironxjsk* :hand-angle-vector :lhand (v+ (send *hironxjsk* :hand-angle-vector :lhand) #f(-7 7 7 -7)))
  (send-left-hand 2.0)
  ;;左右寄せ
  (send *hironxjsk* :larm :move-end-pos (float-vector 0 (- center_y goal_y) 0) :world)
  (cond ((> (- goal_y center_y) 0)
	 (send *hironxjsk* :larm :move-end-pos #f(0 -5 0) :world))
	((< (- goal_y center_y) 0)
	 (send *hironxjsk* :larm :move-end-pos #f(0 5 0) :world)))
  (send-robot)
  (send *hironxjsk* :larm :move-end-pos #f(0 0 70) :world)
  (send-robot)
  )

;;(defun push ()
  ;;To Do
;;  )

(defun place-in ()
  ;;傾ける
  (setq *hiro-coords* (send (send (send *hironxjsk* :rarm :end-coords) :copy-worldcoords) :pos))
  (setq tong-coords (make-cascoords :pos (v- *hiro-coords* *tong-offset*)))
  (send (send *hironxjsk* :rarm :end-coords) :assoc tong-coords)
  (setq target-tong-coords (send tong-coords :copy-worldcoords))
  (send target-tong-coords :rotate -170 :x :world)
  (send *hironxjsk* :rarm :inverse-kinematics target-tong-coords :move-target tong-coords)
  ;;(send *hironxjsk* :rarm :move-end-pos #f(-10 0 0) :world)
  (send-robot)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 -45) :world)
  (send-robot)
  ;;open tongs
  (setq cur_hand (send *ri* :hand-angle-vector :rhand))
  (send *hironxjsk* :hand-angle-vector :rhand (v+ cur_hand #f(10 -10 -10 10)))
  (send-right-hand 1.0)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 45) :world)
  (send-robot)
  ;;元に戻す
  (send target-tong-coords :rotate 170 :x :world)
  (send *hironxjsk* :rarm :inverse-kinematics target-tong-coords :move-target tong-coords)
  (send-robot)
  )

(defun place_cb (msg)
  (setq placed_x (send msg :x))
  (setq placed_y (send msg :y))
  (setq catch_flag t)
  )
  
(defun place-food (pos name food-size)
  ;;same pose
  (setq *hiro-pose* (send (send *hironxjsk* :rarm :end-coords) :copy-worldcoords))
  (setq goal (float-vector (elt pos 0) (* (elt pos 1) -1) (elt pos 2)))
  (setq goal (v+ pos (float-vector *front* *left* 0))) ;;*coords*<-lunchbox pos
  ;;お弁当箱の真ん中に入れてから幅寄せする
  (setq lbox_pos (float-vector (/ (+ *front* back) 2) (/ (+ *left* right) 2) 0))
  ;;(setq lpos (v+ lbox_pos *tong-offset*))
  (setq lpos (v+ lbox_pos (v+ *tong-offset* #f(25 25 0)))) ;;tong offset
  (setq lpos (v+ lpos (float-vector 0 0 (- *catch-z* (elt lpos 2))))) ;;set z *catch-z*
  (setq *move* (v- lpos (send *hiro-pose* :pos)))
  (send *hiro-pose* :translate *move* :world)
  (send *hironxjsk* :rarm :inverse-kinematics *hiro-pose*)
  (send *hironxjsk* :rarm :move-end-pos #f(0 0 80) :world)
  (send-robot)
  (place-in)
  ;;手をよけて置かれた位置を認識する
  (send *hironxjsk* :rarm :move-end-pos #f(0 -200 0) :world)
  (send-robot)
  (sub-img 1)
  (ros::subscribe "/lunchbox_positions/output" geometry_msgs::Point #'place_cb)
  (setq catch_flag nil)
  (until catch_flag
	 (ros::publish "/before_place_img" *bimg-msg*)
	 (ros::publish "/placed_img" *aimg-msg*)
	 (ros::spin-once)
	 (ros::sleep))
  (ros::unsubscribe "/lunchbox_positions/output")
  (setq diff (v- (float-vector placed_x placed_y) lbox-center))
  (format t "lbox-center ~A placed_x, placed_y ~A ~A~%" lbox-center placed_x placed_y)
  (pull_over pos food-size lbox_pos diff)
  ;;(push)
  (place-init-pose)
  )

(defun catch_and_place_main()
  (setq *tong-offset* #f(-20.23 -114.702 90.2251))
  (setq *chopstick-offset* #f(-12.624 18.437 84.2388))
  (format t "- - - CATCH AND PLACE - - -~%")
  (place-init-pose)
  (dotimes (i (length order_list))
    (setq num (elt order_list i))
    (format t "num : ~A~%" num)
    (when (> (elt (elt place_list num) 0) 0)
      (setq target_name (elt name_list num))
      (setq food-size (elt box_list num))
      (format t "target : ~A~%" target_name)
      (format t "size : ~A place : ~A~%" food-size (elt place_list num))
      (if (or (> (elt food-size 0) 35) (> (elt food-size 1) 35))
	  (progn
	    (if (or (> (elt food-size 0) 45) (> (elt food-size 1) 45))
		(open-tongs 2)
	      (open-tongs 1)))
	(open-tongs 0))
      (sub-img 0)
      (catch-food (elt pos_list num) (elt radi_list num))
      (place-food (elt place_list num) target_name food-size)
      )
    )
  )

;;#f(-110.25 9.0407 64.6251) hand-pos - real-pos(rarm)
;;tong(rotate 90)
;;#f(-39.159 -114.508 100.161)

;;11/25トング逆向きにした
;;#f(-22.0 -114.0 100.161)

;;指の開き方
;;#f(-15.449 15.449 15.449 -15.449)) -> 2cm
;;#f(0 0 0 0) -> 4.3cm
;;#f(15.449 -15.449 -15.449 15.449) -> 6.4cm
