; Auto-generated. Do not edit!


(cl:in-package state_estimator_msgs-msg)


;//! \htmlinclude timeSync.msg.html

(cl:defclass <timeSync> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass timeSync (<timeSync>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <timeSync>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'timeSync)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator_msgs-msg:<timeSync> is deprecated: use state_estimator_msgs-msg:timeSync instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <timeSync>) ostream)
  "Serializes a message object of type '<timeSync>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <timeSync>) istream)
  "Deserializes a message object of type '<timeSync>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<timeSync>)))
  "Returns string type for a message object of type '<timeSync>"
  "state_estimator_msgs/timeSync")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'timeSync)))
  "Returns string type for a message object of type 'timeSync"
  "state_estimator_msgs/timeSync")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<timeSync>)))
  "Returns md5sum for a message object of type '<timeSync>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'timeSync)))
  "Returns md5sum for a message object of type 'timeSync"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<timeSync>)))
  "Returns full string definition for message of type '<timeSync>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'timeSync)))
  "Returns full string definition for message of type 'timeSync"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <timeSync>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <timeSync>))
  "Converts a ROS message object to a list"
  (cl:list 'timeSync
))
