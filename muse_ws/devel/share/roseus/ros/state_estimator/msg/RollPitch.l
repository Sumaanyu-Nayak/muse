;; Auto-generated. Do not edit!


(when (boundp 'state_estimator::RollPitch)
  (if (not (find-package "STATE_ESTIMATOR"))
    (make-package "STATE_ESTIMATOR"))
  (shadow 'RollPitch (find-package "STATE_ESTIMATOR")))
(unless (find-package "STATE_ESTIMATOR::ROLLPITCH")
  (make-package "STATE_ESTIMATOR::ROLLPITCH"))

(in-package "ROS")
;;//! \htmlinclude RollPitch.msg.html


(defclass state_estimator::RollPitch
  :super ros::object
  :slots (_roll _pitch ))

(defmethod state_estimator::RollPitch
  (:init
   (&key
    ((:roll __roll) 0.0)
    ((:pitch __pitch) 0.0)
    )
   (send-super :init)
   (setq _roll (float __roll))
   (setq _pitch (float __pitch))
   self)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:serialization-length
   ()
   (+
    ;; float64 _roll
    8
    ;; float64 _pitch
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _roll
     (setq _roll (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pitch
     (setq _pitch (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get state_estimator::RollPitch :md5sum-) "0f5aa311af37faead3f9e90f420c9603")
(setf (get state_estimator::RollPitch :datatype-) "state_estimator/RollPitch")
(setf (get state_estimator::RollPitch :definition-)
      "float64 roll
float64 pitch

")



(provide :state_estimator/RollPitch "0f5aa311af37faead3f9e90f420c9603")


