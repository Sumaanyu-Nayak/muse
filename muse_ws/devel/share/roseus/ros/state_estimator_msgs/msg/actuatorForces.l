;; Auto-generated. Do not edit!


(when (boundp 'state_estimator_msgs::actuatorForces)
  (if (not (find-package "STATE_ESTIMATOR_MSGS"))
    (make-package "STATE_ESTIMATOR_MSGS"))
  (shadow 'actuatorForces (find-package "STATE_ESTIMATOR_MSGS")))
(unless (find-package "STATE_ESTIMATOR_MSGS::ACTUATORFORCES")
  (make-package "STATE_ESTIMATOR_MSGS::ACTUATORFORCES"))

(in-package "ROS")
;;//! \htmlinclude actuatorForces.msg.html


(defclass state_estimator_msgs::actuatorForces
  :super ros::object
  :slots (_tau ))

(defmethod state_estimator_msgs::actuatorForces
  (:init
   (&key
    ((:tau __tau) (make-array 12 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _tau __tau)
   self)
  (:tau
   (&optional __tau)
   (if __tau (setq _tau __tau)) _tau)
  (:serialization-length
   ()
   (+
    ;; float64[12] _tau
    (* 8    12)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[12] _tau
     (dotimes (i 12)
       (sys::poke (elt _tau i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[12] _tau
   (dotimes (i (length _tau))
     (setf (elt _tau i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get state_estimator_msgs::actuatorForces :md5sum-) "ba60240ffc5d00ebf671ed99a32a8ab5")
(setf (get state_estimator_msgs::actuatorForces :datatype-) "state_estimator_msgs/actuatorForces")
(setf (get state_estimator_msgs::actuatorForces :definition-)
      "float64[12] tau

")



(provide :state_estimator_msgs/actuatorForces "ba60240ffc5d00ebf671ed99a32a8ab5")


