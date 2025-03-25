;; Auto-generated. Do not edit!


(when (boundp 'state_estimator_msgs::imu)
  (if (not (find-package "STATE_ESTIMATOR_MSGS"))
    (make-package "STATE_ESTIMATOR_MSGS"))
  (shadow 'imu (find-package "STATE_ESTIMATOR_MSGS")))
(unless (find-package "STATE_ESTIMATOR_MSGS::IMU")
  (make-package "STATE_ESTIMATOR_MSGS::IMU"))

(in-package "ROS")
;;//! \htmlinclude imu.msg.html


(defclass state_estimator_msgs::imu
  :super ros::object
  :slots (_w _a ))

(defmethod state_estimator_msgs::imu
  (:init
   (&key
    ((:w __w) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:a __a) (make-array 3 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _w __w)
   (setq _a __a)
   self)
  (:w
   (&optional __w)
   (if __w (setq _w __w)) _w)
  (:a
   (&optional __a)
   (if __a (setq _a __a)) _a)
  (:serialization-length
   ()
   (+
    ;; float64[3] _w
    (* 8    3)
    ;; float64[3] _a
    (* 8    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[3] _w
     (dotimes (i 3)
       (sys::poke (elt _w i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _a
     (dotimes (i 3)
       (sys::poke (elt _a i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[3] _w
   (dotimes (i (length _w))
     (setf (elt _w i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _a
   (dotimes (i (length _a))
     (setf (elt _a i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get state_estimator_msgs::imu :md5sum-) "3eb7a1d5e42bc0c9297c14a9acbb436b")
(setf (get state_estimator_msgs::imu :datatype-) "state_estimator_msgs/imu")
(setf (get state_estimator_msgs::imu :definition-)
      "float64[3] w
float64[3] a

")



(provide :state_estimator_msgs/imu "3eb7a1d5e42bc0c9297c14a9acbb436b")


