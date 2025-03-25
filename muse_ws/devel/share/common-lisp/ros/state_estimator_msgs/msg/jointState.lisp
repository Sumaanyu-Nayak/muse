; Auto-generated. Do not edit!


(cl:in-package state_estimator_msgs-msg)


;//! \htmlinclude jointState.msg.html

(cl:defclass <jointState> (roslisp-msg-protocol:ros-message)
  ((q
    :reader q
    :initarg :q
    :type (cl:vector cl:float)
   :initform (cl:make-array 12 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass jointState (<jointState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <jointState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'jointState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator_msgs-msg:<jointState> is deprecated: use state_estimator_msgs-msg:jointState instead.")))

(cl:ensure-generic-function 'q-val :lambda-list '(m))
(cl:defmethod q-val ((m <jointState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:q-val is deprecated.  Use state_estimator_msgs-msg:q instead.")
  (q m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <jointState>) ostream)
  "Serializes a message object of type '<jointState>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'q))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <jointState>) istream)
  "Deserializes a message object of type '<jointState>"
  (cl:setf (cl:slot-value msg 'q) (cl:make-array 12))
  (cl:let ((vals (cl:slot-value msg 'q)))
    (cl:dotimes (i 12)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<jointState>)))
  "Returns string type for a message object of type '<jointState>"
  "state_estimator_msgs/jointState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'jointState)))
  "Returns string type for a message object of type 'jointState"
  "state_estimator_msgs/jointState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<jointState>)))
  "Returns md5sum for a message object of type '<jointState>"
  "0439c8b99a1acbdc6519301f07a67111")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'jointState)))
  "Returns md5sum for a message object of type 'jointState"
  "0439c8b99a1acbdc6519301f07a67111")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<jointState>)))
  "Returns full string definition for message of type '<jointState>"
  (cl:format cl:nil "float64[12] q~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'jointState)))
  "Returns full string definition for message of type 'jointState"
  (cl:format cl:nil "float64[12] q~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <jointState>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'q) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <jointState>))
  "Converts a ROS message object to a list"
  (cl:list 'jointState
    (cl:cons ':q (q msg))
))
