;; Auto-generated. Do not edit!


(when (boundp 'state_estimator_msgs::FeetJacobians)
  (if (not (find-package "STATE_ESTIMATOR_MSGS"))
    (make-package "STATE_ESTIMATOR_MSGS"))
  (shadow 'FeetJacobians (find-package "STATE_ESTIMATOR_MSGS")))
(unless (find-package "STATE_ESTIMATOR_MSGS::FEETJACOBIANS")
  (make-package "STATE_ESTIMATOR_MSGS::FEETJACOBIANS"))

(in-package "ROS")
;;//! \htmlinclude FeetJacobians.msg.html


(defclass state_estimator_msgs::FeetJacobians
  :super ros::object
  :slots (_lf _rf _lh _rh ))

(defmethod state_estimator_msgs::FeetJacobians
  (:init
   (&key
    ((:lf __lf) (make-array 9 :initial-element 0.0 :element-type :float))
    ((:rf __rf) (make-array 9 :initial-element 0.0 :element-type :float))
    ((:lh __lh) (make-array 9 :initial-element 0.0 :element-type :float))
    ((:rh __rh) (make-array 9 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _lf __lf)
   (setq _rf __rf)
   (setq _lh __lh)
   (setq _rh __rh)
   self)
  (:lf
   (&optional __lf)
   (if __lf (setq _lf __lf)) _lf)
  (:rf
   (&optional __rf)
   (if __rf (setq _rf __rf)) _rf)
  (:lh
   (&optional __lh)
   (if __lh (setq _lh __lh)) _lh)
  (:rh
   (&optional __rh)
   (if __rh (setq _rh __rh)) _rh)
  (:serialization-length
   ()
   (+
    ;; float64[9] _lf
    (* 8    9)
    ;; float64[9] _rf
    (* 8    9)
    ;; float64[9] _lh
    (* 8    9)
    ;; float64[9] _rh
    (* 8    9)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[9] _lf
     (dotimes (i 9)
       (sys::poke (elt _lf i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[9] _rf
     (dotimes (i 9)
       (sys::poke (elt _rf i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[9] _lh
     (dotimes (i 9)
       (sys::poke (elt _lh i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[9] _rh
     (dotimes (i 9)
       (sys::poke (elt _rh i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[9] _lf
   (dotimes (i (length _lf))
     (setf (elt _lf i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[9] _rf
   (dotimes (i (length _rf))
     (setf (elt _rf i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[9] _lh
   (dotimes (i (length _lh))
     (setf (elt _lh i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[9] _rh
   (dotimes (i (length _rh))
     (setf (elt _rh i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get state_estimator_msgs::FeetJacobians :md5sum-) "c109ea601aeb1745edc38521e6c4252c")
(setf (get state_estimator_msgs::FeetJacobians :datatype-) "state_estimator_msgs/FeetJacobians")
(setf (get state_estimator_msgs::FeetJacobians :definition-)
      "float64[9] lf
float64[9] rf
float64[9] lh
float64[9] rh


")



(provide :state_estimator_msgs/FeetJacobians "c109ea601aeb1745edc38521e6c4252c")


