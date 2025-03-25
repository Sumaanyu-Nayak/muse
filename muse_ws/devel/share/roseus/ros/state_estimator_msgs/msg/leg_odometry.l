;; Auto-generated. Do not edit!


(when (boundp 'state_estimator_msgs::leg_odometry)
  (if (not (find-package "STATE_ESTIMATOR_MSGS"))
    (make-package "STATE_ESTIMATOR_MSGS"))
  (shadow 'leg_odometry (find-package "STATE_ESTIMATOR_MSGS")))
(unless (find-package "STATE_ESTIMATOR_MSGS::LEG_ODOMETRY")
  (make-package "STATE_ESTIMATOR_MSGS::LEG_ODOMETRY"))

(in-package "ROS")
;;//! \htmlinclude leg_odometry.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass state_estimator_msgs::leg_odometry
  :super ros::object
  :slots (_header _lin_vel_lf _lin_vel_rf _lin_vel_lh _lin_vel_rh _base_velocity _stance_lf _stance_rf _stance_lh _stance_rh ))

(defmethod state_estimator_msgs::leg_odometry
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:lin_vel_lf __lin_vel_lf) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:lin_vel_rf __lin_vel_rf) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:lin_vel_lh __lin_vel_lh) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:lin_vel_rh __lin_vel_rh) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:base_velocity __base_velocity) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:stance_lf __stance_lf) nil)
    ((:stance_rf __stance_rf) nil)
    ((:stance_lh __stance_lh) nil)
    ((:stance_rh __stance_rh) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _lin_vel_lf __lin_vel_lf)
   (setq _lin_vel_rf __lin_vel_rf)
   (setq _lin_vel_lh __lin_vel_lh)
   (setq _lin_vel_rh __lin_vel_rh)
   (setq _base_velocity __base_velocity)
   (setq _stance_lf __stance_lf)
   (setq _stance_rf __stance_rf)
   (setq _stance_lh __stance_lh)
   (setq _stance_rh __stance_rh)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:lin_vel_lf
   (&optional __lin_vel_lf)
   (if __lin_vel_lf (setq _lin_vel_lf __lin_vel_lf)) _lin_vel_lf)
  (:lin_vel_rf
   (&optional __lin_vel_rf)
   (if __lin_vel_rf (setq _lin_vel_rf __lin_vel_rf)) _lin_vel_rf)
  (:lin_vel_lh
   (&optional __lin_vel_lh)
   (if __lin_vel_lh (setq _lin_vel_lh __lin_vel_lh)) _lin_vel_lh)
  (:lin_vel_rh
   (&optional __lin_vel_rh)
   (if __lin_vel_rh (setq _lin_vel_rh __lin_vel_rh)) _lin_vel_rh)
  (:base_velocity
   (&optional __base_velocity)
   (if __base_velocity (setq _base_velocity __base_velocity)) _base_velocity)
  (:stance_lf
   (&optional (__stance_lf :null))
   (if (not (eq __stance_lf :null)) (setq _stance_lf __stance_lf)) _stance_lf)
  (:stance_rf
   (&optional (__stance_rf :null))
   (if (not (eq __stance_rf :null)) (setq _stance_rf __stance_rf)) _stance_rf)
  (:stance_lh
   (&optional (__stance_lh :null))
   (if (not (eq __stance_lh :null)) (setq _stance_lh __stance_lh)) _stance_lh)
  (:stance_rh
   (&optional (__stance_rh :null))
   (if (not (eq __stance_rh :null)) (setq _stance_rh __stance_rh)) _stance_rh)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[3] _lin_vel_lf
    (* 8    3)
    ;; float64[3] _lin_vel_rf
    (* 8    3)
    ;; float64[3] _lin_vel_lh
    (* 8    3)
    ;; float64[3] _lin_vel_rh
    (* 8    3)
    ;; float64[3] _base_velocity
    (* 8    3)
    ;; bool _stance_lf
    1
    ;; bool _stance_rf
    1
    ;; bool _stance_lh
    1
    ;; bool _stance_rh
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[3] _lin_vel_lf
     (dotimes (i 3)
       (sys::poke (elt _lin_vel_lf i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _lin_vel_rf
     (dotimes (i 3)
       (sys::poke (elt _lin_vel_rf i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _lin_vel_lh
     (dotimes (i 3)
       (sys::poke (elt _lin_vel_lh i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _lin_vel_rh
     (dotimes (i 3)
       (sys::poke (elt _lin_vel_rh i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _base_velocity
     (dotimes (i 3)
       (sys::poke (elt _base_velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _stance_lf
       (if _stance_lf (write-byte -1 s) (write-byte 0 s))
     ;; bool _stance_rf
       (if _stance_rf (write-byte -1 s) (write-byte 0 s))
     ;; bool _stance_lh
       (if _stance_lh (write-byte -1 s) (write-byte 0 s))
     ;; bool _stance_rh
       (if _stance_rh (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[3] _lin_vel_lf
   (dotimes (i (length _lin_vel_lf))
     (setf (elt _lin_vel_lf i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _lin_vel_rf
   (dotimes (i (length _lin_vel_rf))
     (setf (elt _lin_vel_rf i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _lin_vel_lh
   (dotimes (i (length _lin_vel_lh))
     (setf (elt _lin_vel_lh i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _lin_vel_rh
   (dotimes (i (length _lin_vel_rh))
     (setf (elt _lin_vel_rh i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _base_velocity
   (dotimes (i (length _base_velocity))
     (setf (elt _base_velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; bool _stance_lf
     (setq _stance_lf (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _stance_rf
     (setq _stance_rf (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _stance_lh
     (setq _stance_lh (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _stance_rh
     (setq _stance_rh (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get state_estimator_msgs::leg_odometry :md5sum-) "1d1be3f0837dc21a71a6cc54730caa1c")
(setf (get state_estimator_msgs::leg_odometry :datatype-) "state_estimator_msgs/leg_odometry")
(setf (get state_estimator_msgs::leg_odometry :definition-)
      "std_msgs/Header header

float64[3] lin_vel_lf
float64[3] lin_vel_rf
float64[3] lin_vel_lh
float64[3] lin_vel_rh

float64[3] base_velocity

bool stance_lf
bool stance_rf
bool stance_lh
bool stance_rh

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

")



(provide :state_estimator_msgs/leg_odometry "1d1be3f0837dc21a71a6cc54730caa1c")


