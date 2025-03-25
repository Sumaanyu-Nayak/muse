; Auto-generated. Do not edit!


(cl:in-package state_estimator-msg)


;//! \htmlinclude RollPitchStamped.msg.html

(cl:defclass <RollPitchStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (rollpitch
    :reader rollpitch
    :initarg :rollpitch
    :type state_estimator-msg:RollPitch
    :initform (cl:make-instance 'state_estimator-msg:RollPitch)))
)

(cl:defclass RollPitchStamped (<RollPitchStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RollPitchStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RollPitchStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator-msg:<RollPitchStamped> is deprecated: use state_estimator-msg:RollPitchStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RollPitchStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator-msg:header-val is deprecated.  Use state_estimator-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'rollpitch-val :lambda-list '(m))
(cl:defmethod rollpitch-val ((m <RollPitchStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator-msg:rollpitch-val is deprecated.  Use state_estimator-msg:rollpitch instead.")
  (rollpitch m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RollPitchStamped>) ostream)
  "Serializes a message object of type '<RollPitchStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'rollpitch) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RollPitchStamped>) istream)
  "Deserializes a message object of type '<RollPitchStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'rollpitch) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RollPitchStamped>)))
  "Returns string type for a message object of type '<RollPitchStamped>"
  "state_estimator/RollPitchStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RollPitchStamped)))
  "Returns string type for a message object of type 'RollPitchStamped"
  "state_estimator/RollPitchStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RollPitchStamped>)))
  "Returns md5sum for a message object of type '<RollPitchStamped>"
  "6b6120207ee622e95692282f4b59fcc9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RollPitchStamped)))
  "Returns md5sum for a message object of type 'RollPitchStamped"
  "6b6120207ee622e95692282f4b59fcc9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RollPitchStamped>)))
  "Returns full string definition for message of type '<RollPitchStamped>"
  (cl:format cl:nil "Header header~%RollPitch rollpitch~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: state_estimator/RollPitch~%float64 roll~%float64 pitch~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RollPitchStamped)))
  "Returns full string definition for message of type 'RollPitchStamped"
  (cl:format cl:nil "Header header~%RollPitch rollpitch~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: state_estimator/RollPitch~%float64 roll~%float64 pitch~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RollPitchStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'rollpitch))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RollPitchStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'RollPitchStamped
    (cl:cons ':header (header msg))
    (cl:cons ':rollpitch (rollpitch msg))
))
