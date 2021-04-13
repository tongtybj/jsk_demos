(load "package://spoteus/spot-interface.l")

(spot-init nil) ;; do not create-viewer

(setq *path* (ros::get-param "~path" "/home/spot/autowalk_files/2021-04-12_eng2_73b2_to_81c1_night.walk"))
(setq *init-waypoint* (floor (ros::get-param "~init_waypoint" 0)))
(setq *upload* (ros::get-param "~upload" t))

(if *upload*
    (send *ri* :upload-path *path*))

(ros::ros-info "initialize position with waypoint of ~A" *init-waypoint*)
(send *ri* :initial-localization-waypoint *init-waypoint*)

;; for demo
(ros::ros-info "ready go to 81C1?")
(if (y-or-n-p) t (exit))
(ros::ros-info "OK!")

;; go to 81C1
(send *ri* :navigate-to -1)

;; for demo
(ros::ros-info "start barometer measurement?")
(if (y-or-n-p) t (exit))

(setq srvname "/barometer_measure")
(ros::wait-for-service srvname))
(setq r (ros::service-call srvname (instance std_srvs::SetBoolRequest :init :data t)))
(ros::ros-info "Call \"~A\" returns \"~A\"" srvname (send r :message))

;; for demo
(ros::ros-info "ready go back to 73B2?")
(if (y-or-n-p) t (exit))

(ros::ros-info "OK!")

(ros::wait-for-service srvname))
(setq r (ros::service-call srvname (instance std_srvs::SetBoolRequest :init :data nil)))
(ros::ros-info "Call \"~A\" returns \"~A\"" srvname (send r :message))


;; (exit) ;; force quit for safty

;; go back to 73B2
(send *ri* :navigate-to 0)



