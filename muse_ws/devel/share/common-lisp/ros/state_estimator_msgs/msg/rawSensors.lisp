; Auto-generated. Do not edit!


(cl:in-package state_estimator_msgs-msg)


;//! \htmlinclude rawSensors.msg.html

(cl:defclass <rawSensors> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (tau
    :reader tau
    :initarg :tau
    :type state_estimator_msgs-msg:actuatorForces
    :initform (cl:make-instance 'state_estimator_msgs-msg:actuatorForces))
   (imu
    :reader imu
    :initarg :imu
    :type state_estimator_msgs-msg:imu
    :initform (cl:make-instance 'state_estimator_msgs-msg:imu))
   (imu_secondary
    :reader imu_secondary
    :initarg :imu_secondary
    :type state_estimator_msgs-msg:imu
    :initform (cl:make-instance 'state_estimator_msgs-msg:imu))
   (time_sync
    :reader time_sync
    :initarg :time_sync
    :type state_estimator_msgs-msg:timeSync
    :initform (cl:make-instance 'state_estimator_msgs-msg:timeSync)))
)

(cl:defclass rawSensors (<rawSensors>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rawSensors>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rawSensors)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator_msgs-msg:<rawSensors> is deprecated: use state_estimator_msgs-msg:rawSensors instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <rawSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:header-val is deprecated.  Use state_estimator_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'tau-val :lambda-list '(m))
(cl:defmethod tau-val ((m <rawSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:tau-val is deprecated.  Use state_estimator_msgs-msg:tau instead.")
  (tau m))

(cl:ensure-generic-function 'imu-val :lambda-list '(m))
(cl:defmethod imu-val ((m <rawSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:imu-val is deprecated.  Use state_estimator_msgs-msg:imu instead.")
  (imu m))

(cl:ensure-generic-function 'imu_secondary-val :lambda-list '(m))
(cl:defmethod imu_secondary-val ((m <rawSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:imu_secondary-val is deprecated.  Use state_estimator_msgs-msg:imu_secondary instead.")
  (imu_secondary m))

(cl:ensure-generic-function 'time_sync-val :lambda-list '(m))
(cl:defmethod time_sync-val ((m <rawSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:time_sync-val is deprecated.  Use state_estimator_msgs-msg:time_sync instead.")
  (time_sync m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rawSensors>) ostream)
  "Serializes a message object of type '<rawSensors>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'tau) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'imu) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'imu_secondary) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'time_sync) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rawSensors>) istream)
  "Deserializes a message object of type '<rawSensors>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'tau) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'imu) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'imu_secondary) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'time_sync) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rawSensors>)))
  "Returns string type for a message object of type '<rawSensors>"
  "state_estimator_msgs/rawSensors")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rawSensors)))
  "Returns string type for a message object of type 'rawSensors"
  "state_estimator_msgs/rawSensors")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rawSensors>)))
  "Returns md5sum for a message object of type '<rawSensors>"
  "1bf8247f04179d165127b1c3762c3128")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rawSensors)))
  "Returns md5sum for a message object of type 'rawSensors"
  "1bf8247f04179d165127b1c3762c3128")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rawSensors>)))
  "Returns full string definition for message of type '<rawSensors>"
  (cl:format cl:nil "Header header~%actuatorForces tau~%imu imu~%imu imu_secondary~%timeSync time_sync~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: state_estimator_msgs/actuatorForces~%float64[12] tau~%~%================================================================================~%MSG: state_estimator_msgs/imu~%float64[3] w~%float64[3] a~%~%================================================================================~%MSG: state_estimator_msgs/timeSync~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rawSensors)))
  "Returns full string definition for message of type 'rawSensors"
  (cl:format cl:nil "Header header~%actuatorForces tau~%imu imu~%imu imu_secondary~%timeSync time_sync~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: state_estimator_msgs/actuatorForces~%float64[12] tau~%~%================================================================================~%MSG: state_estimator_msgs/imu~%float64[3] w~%float64[3] a~%~%================================================================================~%MSG: state_estimator_msgs/timeSync~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rawSensors>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'tau))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'imu))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'imu_secondary))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'time_sync))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rawSensors>))
  "Converts a ROS message object to a list"
  (cl:list 'rawSensors
    (cl:cons ':header (header msg))
    (cl:cons ':tau (tau msg))
    (cl:cons ':imu (imu msg))
    (cl:cons ':imu_secondary (imu_secondary msg))
    (cl:cons ':time_sync (time_sync msg))
))
