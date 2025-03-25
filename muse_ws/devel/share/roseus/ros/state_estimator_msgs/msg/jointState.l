;; Auto-generated. Do not edit!


(when (boundp 'state_estimator_msgs::jointState)
  (if (not (find-package "STATE_ESTIMATOR_MSGS"))
    (make-package "STATE_ESTIMATOR_MSGS"))
  (shadow 'jointState (find-package "STATE_ESTIMATOR_MSGS")))
(unless (find-package "STATE_ESTIMATOR_MSGS::JOINTSTATE")
  (make-package "STATE_ESTIMATOR_MSGS::JOINTSTATE"))

(in-package "ROS")
;;//! \htmlinclude jointState.msg.html


(defclass state_estimator_msgs::jointState
  :super ros::object
  :slots (_q ))

(defmethod state_estimator_msgs::jointState
  (:init
   (&key
    ((:q __q) (make-array 12 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _q __q)
   self)
  (:q
   (&optional __q)
   (if __q (setq _q __q)) _q)
  (:serialization-length
   ()
   (+
    ;; float64[12] _q
    (* 8    12)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[12] _q
     (dotimes (i 12)
       (sys::poke (elt _q i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[12] _q
   (dotimes (i (length _q))
     (setf (elt _q i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get state_estimator_msgs::jointState :md5sum-) "0439c8b99a1acbdc6519301f07a67111")
(setf (get state_estimator_msgs::jointState :datatype-) "state_estimator_msgs/jointState")
(setf (get state_estimator_msgs::jointState :definition-)
      "float64[12] q

")



(provide :state_estimator_msgs/jointState "0439c8b99a1acbdc6519301f07a67111")


