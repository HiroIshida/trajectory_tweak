(load "package://pr2eus/pr2-interface.l")
(load "package://roseus_mongo/euslisp/json/json-encode.l")
(load "package://roseus_mongo/euslisp/json/json-decode.l")
(load "utils.l")
(ros::load-ros-manifest "oven")

(ros::load-ros-manifest "roseus")

(ros::roseus "tmp" :anonymous t)
(pr2-init)
(setq *home-av* #f(80.5 64.9444 32.2936 135.609 -107.084 -98.598 -5.72958 271.189 -68.9686 54.907 -40.3966 -93.0573 -129.817 -27.5274 -204.232 1.5487 30.2702))


(defun request_json (name) 
  (let ((service-name name)
        msg json)
      (ros::wait-for-service service-name)
      (setq msg (ros::service-call service-name (instance oven::JsonStringRequest :init)))
      (setq json (json::parse-from-string (send msg :message)))
      json))

(setq *json* (request_json "get_tweak"))

(defun follow-json (json)
  (let ((av-seq (cdr (assoc :av_seq *json*)))
        (T-rt-seq (cdr (assoc :T_rt_seq *json*))))
    (dolist (pair (zip av-seq T-rt-seq))
      (solve-ik (car pair) (cdr pair))
      (send *ri* :angle-vector (send *pr2* :angle-vector))
      (unix:usleep (round (* 0.5 1000 2000))))))

(follow-json *json*)


