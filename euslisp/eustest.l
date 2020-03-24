(load "package://pr2eus/pr2-interface.l")
(load "package://roseus_mongo/euslisp/json/json-encode.l")
(load "package://roseus_mongo/euslisp/json/json-decode.l")
(load "utils.l")
(ros::load-ros-manifest "oven")

(ros::load-ros-manifest "roseus")

(ros::roseus "tmp" :anonymous t)
(pr2-init)
(setq *home-av* #f(80.5 64.9444 32.2936 135.609 -107.084 -98.598 -5.72958 271.189 -68.9686 54.907 -40.3966 -93.0573 -129.817 -27.5274 -204.232 1.5487 30.2702))

(defun request_json (service-name req-json)
  (let (req msg json)
      (ros::wait-for-service service-name)
      (setq req (instance oven::JsonStringRequest :init))
      (send req :message req-json)
      (setq msg (ros::service-call service-name req))
      (setq json (json::parse-from-string (send msg :message)))
      json))

  (setq msg (ros::service-call "get_tweak" (instance oven::JsonStringRequest :init)))

(setq *req-json* "{\"name\": \"test\", \"param\": [0,0,0,0,0,0,0,0,0,0,0,0]}")
;(setq *req-json* "{\"name\": \"test\"}")
(setq *json* (request_json "get_tweak" *req-json*))

(defun follow-json (json)
  (let ((av-seq (cdr (assoc :av_seq *json*)))
        (T-rt-seq (cdr (assoc :T_rt_seq *json*))))
    (dolist (pair (zip av-seq T-rt-seq))
      (solve-ik (car pair) (cdr pair))
      (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
      (unix:sleep 1))))

(follow-json *json*)



