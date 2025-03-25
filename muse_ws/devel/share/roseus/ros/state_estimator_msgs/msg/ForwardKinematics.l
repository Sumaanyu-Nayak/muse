;; Auto-generated. Do not edit!


(when (boundp 'state_estimator_msgs::ForwardKinematics)
  (if (not (find-package "STATE_ESTIMATOR_MSGS"))
    (make-package "STATE_ESTIMATOR_MSGS"))
  (shadow 'ForwardKinematics (find-package "STATE_ESTIMATOR_MSGS")))
(unless (find-package "STATE_ESTIMATOR_MSGS::FORWARDKINEMATICS")
  (make-package "STATE_ESTIMATOR_MSGS::FORWARDKINEMATICS"))

(in-package "ROS")
;;//! \htmlinclude ForwardKinematics.msg.html


(defclass state_estimator_msgs::ForwardKinematics
  :super ros::object
  :slots (_lf_haa _rf_haa _lh_haa _rh_haa _lf_hfe _rf_hfe _lh_hfe _rh_hfe _lf_kfe _rf_kfe _lh_kfe _rh_kfe _lf_foot _rf_foot _lh_foot _rh_foot ))

(defmethod state_estimator_msgs::ForwardKinematics
  (:init
   (&key
    ((:lf_haa __lf_haa) (make-array 16 :initial-element 0.0 :element-type :float))
    ((:rf_haa __rf_haa) (make-array 16 :initial-element 0.0 :element-type :float))
    ((:lh_haa __lh_haa) (make-array 16 :initial-element 0.0 :element-type :float))
    ((:rh_haa __rh_haa) (make-array 16 :initial-element 0.0 :element-type :float))
    ((:lf_hfe __lf_hfe) (make-array 16 :initial-element 0.0 :element-type :float))
    ((:rf_hfe __rf_hfe) (make-array 16 :initial-element 0.0 :element-type :float))
    ((:lh_hfe __lh_hfe) (make-array 16 :initial-element 0.0 :element-type :float))
    ((:rh_hfe __rh_hfe) (make-array 16 :initial-element 0.0 :element-type :float))
    ((:lf_kfe __lf_kfe) (make-array 16 :initial-element 0.0 :element-type :float))
    ((:rf_kfe __rf_kfe) (make-array 16 :initial-element 0.0 :element-type :float))
    ((:lh_kfe __lh_kfe) (make-array 16 :initial-element 0.0 :element-type :float))
    ((:rh_kfe __rh_kfe) (make-array 16 :initial-element 0.0 :element-type :float))
    ((:lf_foot __lf_foot) (make-array 16 :initial-element 0.0 :element-type :float))
    ((:rf_foot __rf_foot) (make-array 16 :initial-element 0.0 :element-type :float))
    ((:lh_foot __lh_foot) (make-array 16 :initial-element 0.0 :element-type :float))
    ((:rh_foot __rh_foot) (make-array 16 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _lf_haa __lf_haa)
   (setq _rf_haa __rf_haa)
   (setq _lh_haa __lh_haa)
   (setq _rh_haa __rh_haa)
   (setq _lf_hfe __lf_hfe)
   (setq _rf_hfe __rf_hfe)
   (setq _lh_hfe __lh_hfe)
   (setq _rh_hfe __rh_hfe)
   (setq _lf_kfe __lf_kfe)
   (setq _rf_kfe __rf_kfe)
   (setq _lh_kfe __lh_kfe)
   (setq _rh_kfe __rh_kfe)
   (setq _lf_foot __lf_foot)
   (setq _rf_foot __rf_foot)
   (setq _lh_foot __lh_foot)
   (setq _rh_foot __rh_foot)
   self)
  (:lf_haa
   (&optional __lf_haa)
   (if __lf_haa (setq _lf_haa __lf_haa)) _lf_haa)
  (:rf_haa
   (&optional __rf_haa)
   (if __rf_haa (setq _rf_haa __rf_haa)) _rf_haa)
  (:lh_haa
   (&optional __lh_haa)
   (if __lh_haa (setq _lh_haa __lh_haa)) _lh_haa)
  (:rh_haa
   (&optional __rh_haa)
   (if __rh_haa (setq _rh_haa __rh_haa)) _rh_haa)
  (:lf_hfe
   (&optional __lf_hfe)
   (if __lf_hfe (setq _lf_hfe __lf_hfe)) _lf_hfe)
  (:rf_hfe
   (&optional __rf_hfe)
   (if __rf_hfe (setq _rf_hfe __rf_hfe)) _rf_hfe)
  (:lh_hfe
   (&optional __lh_hfe)
   (if __lh_hfe (setq _lh_hfe __lh_hfe)) _lh_hfe)
  (:rh_hfe
   (&optional __rh_hfe)
   (if __rh_hfe (setq _rh_hfe __rh_hfe)) _rh_hfe)
  (:lf_kfe
   (&optional __lf_kfe)
   (if __lf_kfe (setq _lf_kfe __lf_kfe)) _lf_kfe)
  (:rf_kfe
   (&optional __rf_kfe)
   (if __rf_kfe (setq _rf_kfe __rf_kfe)) _rf_kfe)
  (:lh_kfe
   (&optional __lh_kfe)
   (if __lh_kfe (setq _lh_kfe __lh_kfe)) _lh_kfe)
  (:rh_kfe
   (&optional __rh_kfe)
   (if __rh_kfe (setq _rh_kfe __rh_kfe)) _rh_kfe)
  (:lf_foot
   (&optional __lf_foot)
   (if __lf_foot (setq _lf_foot __lf_foot)) _lf_foot)
  (:rf_foot
   (&optional __rf_foot)
   (if __rf_foot (setq _rf_foot __rf_foot)) _rf_foot)
  (:lh_foot
   (&optional __lh_foot)
   (if __lh_foot (setq _lh_foot __lh_foot)) _lh_foot)
  (:rh_foot
   (&optional __rh_foot)
   (if __rh_foot (setq _rh_foot __rh_foot)) _rh_foot)
  (:serialization-length
   ()
   (+
    ;; float64[16] _lf_haa
    (* 8    16)
    ;; float64[16] _rf_haa
    (* 8    16)
    ;; float64[16] _lh_haa
    (* 8    16)
    ;; float64[16] _rh_haa
    (* 8    16)
    ;; float64[16] _lf_hfe
    (* 8    16)
    ;; float64[16] _rf_hfe
    (* 8    16)
    ;; float64[16] _lh_hfe
    (* 8    16)
    ;; float64[16] _rh_hfe
    (* 8    16)
    ;; float64[16] _lf_kfe
    (* 8    16)
    ;; float64[16] _rf_kfe
    (* 8    16)
    ;; float64[16] _lh_kfe
    (* 8    16)
    ;; float64[16] _rh_kfe
    (* 8    16)
    ;; float64[16] _lf_foot
    (* 8    16)
    ;; float64[16] _rf_foot
    (* 8    16)
    ;; float64[16] _lh_foot
    (* 8    16)
    ;; float64[16] _rh_foot
    (* 8    16)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[16] _lf_haa
     (dotimes (i 16)
       (sys::poke (elt _lf_haa i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[16] _rf_haa
     (dotimes (i 16)
       (sys::poke (elt _rf_haa i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[16] _lh_haa
     (dotimes (i 16)
       (sys::poke (elt _lh_haa i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[16] _rh_haa
     (dotimes (i 16)
       (sys::poke (elt _rh_haa i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[16] _lf_hfe
     (dotimes (i 16)
       (sys::poke (elt _lf_hfe i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[16] _rf_hfe
     (dotimes (i 16)
       (sys::poke (elt _rf_hfe i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[16] _lh_hfe
     (dotimes (i 16)
       (sys::poke (elt _lh_hfe i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[16] _rh_hfe
     (dotimes (i 16)
       (sys::poke (elt _rh_hfe i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[16] _lf_kfe
     (dotimes (i 16)
       (sys::poke (elt _lf_kfe i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[16] _rf_kfe
     (dotimes (i 16)
       (sys::poke (elt _rf_kfe i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[16] _lh_kfe
     (dotimes (i 16)
       (sys::poke (elt _lh_kfe i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[16] _rh_kfe
     (dotimes (i 16)
       (sys::poke (elt _rh_kfe i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[16] _lf_foot
     (dotimes (i 16)
       (sys::poke (elt _lf_foot i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[16] _rf_foot
     (dotimes (i 16)
       (sys::poke (elt _rf_foot i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[16] _lh_foot
     (dotimes (i 16)
       (sys::poke (elt _lh_foot i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[16] _rh_foot
     (dotimes (i 16)
       (sys::poke (elt _rh_foot i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[16] _lf_haa
   (dotimes (i (length _lf_haa))
     (setf (elt _lf_haa i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[16] _rf_haa
   (dotimes (i (length _rf_haa))
     (setf (elt _rf_haa i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[16] _lh_haa
   (dotimes (i (length _lh_haa))
     (setf (elt _lh_haa i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[16] _rh_haa
   (dotimes (i (length _rh_haa))
     (setf (elt _rh_haa i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[16] _lf_hfe
   (dotimes (i (length _lf_hfe))
     (setf (elt _lf_hfe i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[16] _rf_hfe
   (dotimes (i (length _rf_hfe))
     (setf (elt _rf_hfe i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[16] _lh_hfe
   (dotimes (i (length _lh_hfe))
     (setf (elt _lh_hfe i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[16] _rh_hfe
   (dotimes (i (length _rh_hfe))
     (setf (elt _rh_hfe i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[16] _lf_kfe
   (dotimes (i (length _lf_kfe))
     (setf (elt _lf_kfe i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[16] _rf_kfe
   (dotimes (i (length _rf_kfe))
     (setf (elt _rf_kfe i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[16] _lh_kfe
   (dotimes (i (length _lh_kfe))
     (setf (elt _lh_kfe i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[16] _rh_kfe
   (dotimes (i (length _rh_kfe))
     (setf (elt _rh_kfe i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[16] _lf_foot
   (dotimes (i (length _lf_foot))
     (setf (elt _lf_foot i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[16] _rf_foot
   (dotimes (i (length _rf_foot))
     (setf (elt _rf_foot i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[16] _lh_foot
   (dotimes (i (length _lh_foot))
     (setf (elt _lh_foot i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[16] _rh_foot
   (dotimes (i (length _rh_foot))
     (setf (elt _rh_foot i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get state_estimator_msgs::ForwardKinematics :md5sum-) "ae408d1e170b8d8b6fa4c83a5b6670c0")
(setf (get state_estimator_msgs::ForwardKinematics :datatype-) "state_estimator_msgs/ForwardKinematics")
(setf (get state_estimator_msgs::ForwardKinematics :definition-)
      "float64[16] lf_haa
float64[16] rf_haa
float64[16] lh_haa
float64[16] rh_haa

float64[16] lf_hfe
float64[16] rf_hfe
float64[16] lh_hfe
float64[16] rh_hfe

float64[16] lf_kfe
float64[16] rf_kfe
float64[16] lh_kfe
float64[16] rh_kfe

float64[16] lf_foot
float64[16] rf_foot
float64[16] lh_foot
float64[16] rh_foot

")



(provide :state_estimator_msgs/ForwardKinematics "ae408d1e170b8d8b6fa4c83a5b6670c0")


