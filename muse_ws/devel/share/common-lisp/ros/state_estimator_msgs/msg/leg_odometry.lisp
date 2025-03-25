; Auto-generated. Do not edit!


(cl:in-package state_estimator_msgs-msg)


;//! \htmlinclude leg_odometry.msg.html

(cl:defclass <leg_odometry> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (lin_vel_lf
    :reader lin_vel_lf
    :initarg :lin_vel_lf
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (lin_vel_rf
    :reader lin_vel_rf
    :initarg :lin_vel_rf
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (lin_vel_lh
    :reader lin_vel_lh
    :initarg :lin_vel_lh
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (lin_vel_rh
    :reader lin_vel_rh
    :initarg :lin_vel_rh
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (base_velocity
    :reader base_velocity
    :initarg :base_velocity
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (stance_lf
    :reader stance_lf
    :initarg :stance_lf
    :type cl:boolean
    :initform cl:nil)
   (stance_rf
    :reader stance_rf
    :initarg :stance_rf
    :type cl:boolean
    :initform cl:nil)
   (stance_lh
    :reader stance_lh
    :initarg :stance_lh
    :type cl:boolean
    :initform cl:nil)
   (stance_rh
    :reader stance_rh
    :initarg :stance_rh
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass leg_odometry (<leg_odometry>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <leg_odometry>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'leg_odometry)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator_msgs-msg:<leg_odometry> is deprecated: use state_estimator_msgs-msg:leg_odometry instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <leg_odometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:header-val is deprecated.  Use state_estimator_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'lin_vel_lf-val :lambda-list '(m))
(cl:defmethod lin_vel_lf-val ((m <leg_odometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:lin_vel_lf-val is deprecated.  Use state_estimator_msgs-msg:lin_vel_lf instead.")
  (lin_vel_lf m))

(cl:ensure-generic-function 'lin_vel_rf-val :lambda-list '(m))
(cl:defmethod lin_vel_rf-val ((m <leg_odometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:lin_vel_rf-val is deprecated.  Use state_estimator_msgs-msg:lin_vel_rf instead.")
  (lin_vel_rf m))

(cl:ensure-generic-function 'lin_vel_lh-val :lambda-list '(m))
(cl:defmethod lin_vel_lh-val ((m <leg_odometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:lin_vel_lh-val is deprecated.  Use state_estimator_msgs-msg:lin_vel_lh instead.")
  (lin_vel_lh m))

(cl:ensure-generic-function 'lin_vel_rh-val :lambda-list '(m))
(cl:defmethod lin_vel_rh-val ((m <leg_odometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:lin_vel_rh-val is deprecated.  Use state_estimator_msgs-msg:lin_vel_rh instead.")
  (lin_vel_rh m))

(cl:ensure-generic-function 'base_velocity-val :lambda-list '(m))
(cl:defmethod base_velocity-val ((m <leg_odometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:base_velocity-val is deprecated.  Use state_estimator_msgs-msg:base_velocity instead.")
  (base_velocity m))

(cl:ensure-generic-function 'stance_lf-val :lambda-list '(m))
(cl:defmethod stance_lf-val ((m <leg_odometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:stance_lf-val is deprecated.  Use state_estimator_msgs-msg:stance_lf instead.")
  (stance_lf m))

(cl:ensure-generic-function 'stance_rf-val :lambda-list '(m))
(cl:defmethod stance_rf-val ((m <leg_odometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:stance_rf-val is deprecated.  Use state_estimator_msgs-msg:stance_rf instead.")
  (stance_rf m))

(cl:ensure-generic-function 'stance_lh-val :lambda-list '(m))
(cl:defmethod stance_lh-val ((m <leg_odometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:stance_lh-val is deprecated.  Use state_estimator_msgs-msg:stance_lh instead.")
  (stance_lh m))

(cl:ensure-generic-function 'stance_rh-val :lambda-list '(m))
(cl:defmethod stance_rh-val ((m <leg_odometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:stance_rh-val is deprecated.  Use state_estimator_msgs-msg:stance_rh instead.")
  (stance_rh m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <leg_odometry>) ostream)
  "Serializes a message object of type '<leg_odometry>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'lin_vel_lf))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'lin_vel_rf))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'lin_vel_lh))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'lin_vel_rh))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'base_velocity))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'stance_lf) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'stance_rf) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'stance_lh) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'stance_rh) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <leg_odometry>) istream)
  "Deserializes a message object of type '<leg_odometry>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'lin_vel_lf) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'lin_vel_lf)))
    (cl:dotimes (i 3)
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
  (cl:setf (cl:slot-value msg 'lin_vel_rf) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'lin_vel_rf)))
    (cl:dotimes (i 3)
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
  (cl:setf (cl:slot-value msg 'lin_vel_lh) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'lin_vel_lh)))
    (cl:dotimes (i 3)
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
  (cl:setf (cl:slot-value msg 'lin_vel_rh) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'lin_vel_rh)))
    (cl:dotimes (i 3)
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
  (cl:setf (cl:slot-value msg 'base_velocity) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'base_velocity)))
    (cl:dotimes (i 3)
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
    (cl:setf (cl:slot-value msg 'stance_lf) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'stance_rf) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'stance_lh) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'stance_rh) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<leg_odometry>)))
  "Returns string type for a message object of type '<leg_odometry>"
  "state_estimator_msgs/leg_odometry")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'leg_odometry)))
  "Returns string type for a message object of type 'leg_odometry"
  "state_estimator_msgs/leg_odometry")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<leg_odometry>)))
  "Returns md5sum for a message object of type '<leg_odometry>"
  "1d1be3f0837dc21a71a6cc54730caa1c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'leg_odometry)))
  "Returns md5sum for a message object of type 'leg_odometry"
  "1d1be3f0837dc21a71a6cc54730caa1c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<leg_odometry>)))
  "Returns full string definition for message of type '<leg_odometry>"
  (cl:format cl:nil "std_msgs/Header header~%~%float64[3] lin_vel_lf~%float64[3] lin_vel_rf~%float64[3] lin_vel_lh~%float64[3] lin_vel_rh~%~%float64[3] base_velocity~%~%bool stance_lf~%bool stance_rf~%bool stance_lh~%bool stance_rh~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'leg_odometry)))
  "Returns full string definition for message of type 'leg_odometry"
  (cl:format cl:nil "std_msgs/Header header~%~%float64[3] lin_vel_lf~%float64[3] lin_vel_rf~%float64[3] lin_vel_lh~%float64[3] lin_vel_rh~%~%float64[3] base_velocity~%~%bool stance_lf~%bool stance_rf~%bool stance_lh~%bool stance_rh~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <leg_odometry>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'lin_vel_lf) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'lin_vel_rf) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'lin_vel_lh) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'lin_vel_rh) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'base_velocity) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <leg_odometry>))
  "Converts a ROS message object to a list"
  (cl:list 'leg_odometry
    (cl:cons ':header (header msg))
    (cl:cons ':lin_vel_lf (lin_vel_lf msg))
    (cl:cons ':lin_vel_rf (lin_vel_rf msg))
    (cl:cons ':lin_vel_lh (lin_vel_lh msg))
    (cl:cons ':lin_vel_rh (lin_vel_rh msg))
    (cl:cons ':base_velocity (base_velocity msg))
    (cl:cons ':stance_lf (stance_lf msg))
    (cl:cons ':stance_rf (stance_rf msg))
    (cl:cons ':stance_lh (stance_lh msg))
    (cl:cons ':stance_rh (stance_rh msg))
))
