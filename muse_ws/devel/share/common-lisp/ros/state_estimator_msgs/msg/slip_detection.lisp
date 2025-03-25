; Auto-generated. Do not edit!


(cl:in-package state_estimator_msgs-msg)


;//! \htmlinclude slip_detection.msg.html

(cl:defclass <slip_detection> (roslisp-msg-protocol:ros-message)
  ((legs_slipping
    :reader legs_slipping
    :initarg :legs_slipping
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (slip_lf
    :reader slip_lf
    :initarg :slip_lf
    :type cl:boolean
    :initform cl:nil)
   (slip_rf
    :reader slip_rf
    :initarg :slip_rf
    :type cl:boolean
    :initform cl:nil)
   (slip_lh
    :reader slip_lh
    :initarg :slip_lh
    :type cl:boolean
    :initform cl:nil)
   (slip_rh
    :reader slip_rh
    :initarg :slip_rh
    :type cl:boolean
    :initform cl:nil)
   (slippage
    :reader slippage
    :initarg :slippage
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass slip_detection (<slip_detection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <slip_detection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'slip_detection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator_msgs-msg:<slip_detection> is deprecated: use state_estimator_msgs-msg:slip_detection instead.")))

(cl:ensure-generic-function 'legs_slipping-val :lambda-list '(m))
(cl:defmethod legs_slipping-val ((m <slip_detection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:legs_slipping-val is deprecated.  Use state_estimator_msgs-msg:legs_slipping instead.")
  (legs_slipping m))

(cl:ensure-generic-function 'slip_lf-val :lambda-list '(m))
(cl:defmethod slip_lf-val ((m <slip_detection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:slip_lf-val is deprecated.  Use state_estimator_msgs-msg:slip_lf instead.")
  (slip_lf m))

(cl:ensure-generic-function 'slip_rf-val :lambda-list '(m))
(cl:defmethod slip_rf-val ((m <slip_detection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:slip_rf-val is deprecated.  Use state_estimator_msgs-msg:slip_rf instead.")
  (slip_rf m))

(cl:ensure-generic-function 'slip_lh-val :lambda-list '(m))
(cl:defmethod slip_lh-val ((m <slip_detection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:slip_lh-val is deprecated.  Use state_estimator_msgs-msg:slip_lh instead.")
  (slip_lh m))

(cl:ensure-generic-function 'slip_rh-val :lambda-list '(m))
(cl:defmethod slip_rh-val ((m <slip_detection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:slip_rh-val is deprecated.  Use state_estimator_msgs-msg:slip_rh instead.")
  (slip_rh m))

(cl:ensure-generic-function 'slippage-val :lambda-list '(m))
(cl:defmethod slippage-val ((m <slip_detection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:slippage-val is deprecated.  Use state_estimator_msgs-msg:slippage instead.")
  (slippage m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <slip_detection>) ostream)
  "Serializes a message object of type '<slip_detection>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'legs_slipping))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'slip_lf) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'slip_rf) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'slip_lh) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'slip_rh) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'slippage) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <slip_detection>) istream)
  "Deserializes a message object of type '<slip_detection>"
  (cl:setf (cl:slot-value msg 'legs_slipping) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'legs_slipping)))
    (cl:dotimes (i 4)
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
    (cl:setf (cl:slot-value msg 'slip_lf) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'slip_rf) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'slip_lh) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'slip_rh) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'slippage) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<slip_detection>)))
  "Returns string type for a message object of type '<slip_detection>"
  "state_estimator_msgs/slip_detection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'slip_detection)))
  "Returns string type for a message object of type 'slip_detection"
  "state_estimator_msgs/slip_detection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<slip_detection>)))
  "Returns md5sum for a message object of type '<slip_detection>"
  "87252336384194759607e2e036859110")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'slip_detection)))
  "Returns md5sum for a message object of type 'slip_detection"
  "87252336384194759607e2e036859110")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<slip_detection>)))
  "Returns full string definition for message of type '<slip_detection>"
  (cl:format cl:nil "float64[4] legs_slipping~%bool slip_lf~%bool slip_rf~%bool slip_lh~%bool slip_rh~%bool slippage~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'slip_detection)))
  "Returns full string definition for message of type 'slip_detection"
  (cl:format cl:nil "float64[4] legs_slipping~%bool slip_lf~%bool slip_rf~%bool slip_lh~%bool slip_rh~%bool slippage~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <slip_detection>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'legs_slipping) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <slip_detection>))
  "Converts a ROS message object to a list"
  (cl:list 'slip_detection
    (cl:cons ':legs_slipping (legs_slipping msg))
    (cl:cons ':slip_lf (slip_lf msg))
    (cl:cons ':slip_rf (slip_rf msg))
    (cl:cons ':slip_lh (slip_lh msg))
    (cl:cons ':slip_rh (slip_rh msg))
    (cl:cons ':slippage (slippage msg))
))
