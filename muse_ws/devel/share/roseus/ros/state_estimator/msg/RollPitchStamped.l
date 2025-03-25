;; Auto-generated. Do not edit!


(when (boundp 'state_estimator::RollPitchStamped)
  (if (not (find-package "STATE_ESTIMATOR"))
    (make-package "STATE_ESTIMATOR"))
  (shadow 'RollPitchStamped (find-package "STATE_ESTIMATOR")))
(unless (find-package "STATE_ESTIMATOR::ROLLPITCHSTAMPED")
  (make-package "STATE_ESTIMATOR::ROLLPITCHSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude RollPitchStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass state_estimator::RollPitchStamped
  :super ros::object
  :slots (_header _rollpitch ))

(defmethod state_estimator::RollPitchStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:rollpitch __rollpitch) (instance state_estimator::RollPitch :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _rollpitch __rollpitch)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:rollpitch
   (&rest __rollpitch)
   (if (keywordp (car __rollpitch))
       (send* _rollpitch __rollpitch)
     (progn
       (if __rollpitch (setq _rollpitch (car __rollpitch)))
       _rollpitch)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; state_estimator/RollPitch _rollpitch
    (send _rollpitch :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; state_estimator/RollPitch _rollpitch
       (send _rollpitch :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; state_estimator/RollPitch _rollpitch
     (send _rollpitch :deserialize buf ptr-) (incf ptr- (send _rollpitch :serialization-length))
   ;;
   self)
  )

(setf (get state_estimator::RollPitchStamped :md5sum-) "6b6120207ee622e95692282f4b59fcc9")
(setf (get state_estimator::RollPitchStamped :datatype-) "state_estimator/RollPitchStamped")
(setf (get state_estimator::RollPitchStamped :definition-)
      "Header header
RollPitch rollpitch


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: state_estimator/RollPitch
float64 roll
float64 pitch

")



(provide :state_estimator/RollPitchStamped "6b6120207ee622e95692282f4b59fcc9")


