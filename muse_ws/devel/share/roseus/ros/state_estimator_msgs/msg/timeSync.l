;; Auto-generated. Do not edit!


(when (boundp 'state_estimator_msgs::timeSync)
  (if (not (find-package "STATE_ESTIMATOR_MSGS"))
    (make-package "STATE_ESTIMATOR_MSGS"))
  (shadow 'timeSync (find-package "STATE_ESTIMATOR_MSGS")))
(unless (find-package "STATE_ESTIMATOR_MSGS::TIMESYNC")
  (make-package "STATE_ESTIMATOR_MSGS::TIMESYNC"))

(in-package "ROS")
;;//! \htmlinclude timeSync.msg.html


(defclass state_estimator_msgs::timeSync
  :super ros::object
  :slots ())

(defmethod state_estimator_msgs::timeSync
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get state_estimator_msgs::timeSync :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get state_estimator_msgs::timeSync :datatype-) "state_estimator_msgs/timeSync")
(setf (get state_estimator_msgs::timeSync :definition-)
      "
")



(provide :state_estimator_msgs/timeSync "d41d8cd98f00b204e9800998ecf8427e")


