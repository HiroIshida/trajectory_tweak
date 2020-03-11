
(defun zip (lst1 lst2)
  (mapcar #'cons lst1 lst2))

(defun make-quaternion-msg (q-lst)
  (let ((q-msg (instance geometry_msgs::Quaternion :init)))
    (dolist (key-idx (zip '(:x :y :z :w) '(0 1 2 3)))
      (send q-msg (car key-idx) (elt q-lst (cdr key-idx))))
    q-msg))

(defun insert-rarm-av (av-original av-insert)
  (let ((av (copy-object av-original)))
    (dotimes (i 7)
      (let ((idx (+ i 8)))
        (setf (aref av idx) (elt av-insert idx))))
    av))

(defun tf->coords (tf)
  (let ((co (make-coords))
        q-msg
        rot)
        
    (setq q-msg (make-quaternion-msg (second tf)))
    (setq rot (ros::tf-quaternion->rot q-msg))

    (send co :translate (scale 1000 (apply #'float-vector (first  tf))) :world)
    (send co :rotate-with-matrix rot :world)
    co))

(defun solve-ik (av-init tf)
  (send *pr2* :angle-vector (insert-rarm-av *home-av* av-init))
  (send *pr2* :rarm :inverse-kinematics 
        (tf->coords tf)
        :rotation-axis t
        :use-torso nil))


(defun command-following-json (json &optional (duration 2000) (wait nil))
  (let ((av-list
          (mapcar #'(lambda (lst) (apply #'float-vector lst))
                  (cdr (assoc :av_seq json))))
        (tf-bt-list (cdr (assoc :T_rt json))))
    (dolist (av-tf (zip av-list tf-bt-list))
      (solve-ik (car av-tf) (cdr av-tf))
      (send *ri* :angle-vector (send *pr2* :angle-vector) duration)
      (if wait
          (send *ri* :wait-interpolation)
          (unix:usleep (round (* 0.5 1000 duration)))
          )
      )
    ))
