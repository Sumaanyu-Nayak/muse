; Auto-generated. Do not edit!


(cl:in-package state_estimator_msgs-msg)


;//! \htmlinclude actuatorForces.msg.html

(cl:defclass <actuatorForces> (roslisp-msg-protocol:ros-message)
  ((tau
    :reader tau
    :initarg :tau
    :type (cl:vector cl:float)
   :initform (cl:make-array 12 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass actuatorForces (<actuatorForces>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <actuatorForces>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'actuatorForces)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator_msgs-msg:<actuatorForces> is deprecated: use state_estimator_msgs-msg:actuatorForces instead.")))

(cl:ensure-generic-function 'tau-val :lambda-list '(m))
(cl:defmethod tau-val ((m <actuatorForces>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:tau-val is deprecated.  Use state_estimator_msgs-msg:tau instead.")
  (tau m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <actuatorForces>) ostream)
  "Serializes a message object of type '<actuatorForces>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'tau))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <actuatorForces>) istream)
  "Deserializes a message object of type '<actuatorForces>"
  (cl:setf (cl:slot-value msg 'tau) (cl:make-array 12))
  (cl:let ((vals (cl:slot-value msg 'tau)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<actuatorForces>)))
  "Returns string type for a message object of type '<actuatorForces>"
  "state_estimator_msgs/actuatorForces")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'actuatorForces)))
  "Returns string type for a message object of type 'actuatorForces"
  "state_estimator_msgs/actuatorForces")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<actuatorForces>)))
  "Returns md5sum for a message object of type '<actuatorForces>"
  "ba60240ffc5d00ebf671ed99a32a8ab5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'actuatorForces)))
  "Returns md5sum for a message object of type 'actuatorForces"
  "ba60240ffc5d00ebf671ed99a32a8ab5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<actuatorForces>)))
  "Returns full string definition for message of type '<actuatorForces>"
  (cl:format cl:nil "float64[12] tau~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'actuatorForces)))
  "Returns full string definition for message of type 'actuatorForces"
  (cl:format cl:nil "float64[12] tau~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <actuatorForces>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'tau) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <actuatorForces>))
  "Converts a ROS message object to a list"
  (cl:list 'actuatorForces
    (cl:cons ':tau (tau msg))
))
