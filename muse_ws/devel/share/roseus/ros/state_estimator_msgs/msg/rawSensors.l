;; Auto-generated. Do not edit!


(when (boundp 'state_estimator_msgs::rawSensors)
  (if (not (find-package "STATE_ESTIMATOR_MSGS"))
    (make-package "STATE_ESTIMATOR_MSGS"))
  (shadow 'rawSensors (find-package "STATE_ESTIMATOR_MSGS")))
(unless (find-package "STATE_ESTIMATOR_MSGS::RAWSENSORS")
  (make-package "STATE_ESTIMATOR_MSGS::RAWSENSORS"))

(in-package "ROS")
;;//! \htmlinclude rawSensors.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass state_estimator_msgs::rawSensors
  :super ros::object
  :slots (_header _tau _imu _imu_secondary _time_sync ))

(defmethod state_estimator_msgs::rawSensors
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:tau __tau) (instance state_estimator_msgs::actuatorForces :init))
    ((:imu __imu) (instance state_estimator_msgs::imu :init))
    ((:imu_secondary __imu_secondary) (instance state_estimator_msgs::imu :init))
    ((:time_sync __time_sync) (instance state_estimator_msgs::timeSync :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _tau __tau)
   (setq _imu __imu)
   (setq _imu_secondary __imu_secondary)
   (setq _time_sync __time_sync)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:tau
   (&rest __tau)
   (if (keywordp (car __tau))
       (send* _tau __tau)
     (progn
       (if __tau (setq _tau (car __tau)))
       _tau)))
  (:imu
   (&rest __imu)
   (if (keywordp (car __imu))
       (send* _imu __imu)
     (progn
       (if __imu (setq _imu (car __imu)))
       _imu)))
  (:imu_secondary
   (&rest __imu_secondary)
   (if (keywordp (car __imu_secondary))
       (send* _imu_secondary __imu_secondary)
     (progn
       (if __imu_secondary (setq _imu_secondary (car __imu_secondary)))
       _imu_secondary)))
  (:time_sync
   (&rest __time_sync)
   (if (keywordp (car __time_sync))
       (send* _time_sync __time_sync)
     (progn
       (if __time_sync (setq _time_sync (car __time_sync)))
       _time_sync)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; state_estimator_msgs/actuatorForces _tau
    (send _tau :serialization-length)
    ;; state_estimator_msgs/imu _imu
    (send _imu :serialization-length)
    ;; state_estimator_msgs/imu _imu_secondary
    (send _imu_secondary :serialization-length)
    ;; state_estimator_msgs/timeSync _time_sync
    (send _time_sync :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; state_estimator_msgs/actuatorForces _tau
       (send _tau :serialize s)
     ;; state_estimator_msgs/imu _imu
       (send _imu :serialize s)
     ;; state_estimator_msgs/imu _imu_secondary
       (send _imu_secondary :serialize s)
     ;; state_estimator_msgs/timeSync _time_sync
       (send _time_sync :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; state_estimator_msgs/actuatorForces _tau
     (send _tau :deserialize buf ptr-) (incf ptr- (send _tau :serialization-length))
   ;; state_estimator_msgs/imu _imu
     (send _imu :deserialize buf ptr-) (incf ptr- (send _imu :serialization-length))
   ;; state_estimator_msgs/imu _imu_secondary
     (send _imu_secondary :deserialize buf ptr-) (incf ptr- (send _imu_secondary :serialization-length))
   ;; state_estimator_msgs/timeSync _time_sync
     (send _time_sync :deserialize buf ptr-) (incf ptr- (send _time_sync :serialization-length))
   ;;
   self)
  )

(setf (get state_estimator_msgs::rawSensors :md5sum-) "1bf8247f04179d165127b1c3762c3128")
(setf (get state_estimator_msgs::rawSensors :datatype-) "state_estimator_msgs/rawSensors")
(setf (get state_estimator_msgs::rawSensors :definition-)
      "Header header
actuatorForces tau
imu imu
imu imu_secondary
timeSync time_sync

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
MSG: state_estimator_msgs/actuatorForces
float64[12] tau

================================================================================
MSG: state_estimator_msgs/imu
float64[3] w
float64[3] a

================================================================================
MSG: state_estimator_msgs/timeSync

")



(provide :state_estimator_msgs/rawSensors "1bf8247f04179d165127b1c3762c3128")


