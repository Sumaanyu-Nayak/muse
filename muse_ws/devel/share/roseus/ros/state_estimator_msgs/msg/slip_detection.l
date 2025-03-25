;; Auto-generated. Do not edit!


(when (boundp 'state_estimator_msgs::slip_detection)
  (if (not (find-package "STATE_ESTIMATOR_MSGS"))
    (make-package "STATE_ESTIMATOR_MSGS"))
  (shadow 'slip_detection (find-package "STATE_ESTIMATOR_MSGS")))
(unless (find-package "STATE_ESTIMATOR_MSGS::SLIP_DETECTION")
  (make-package "STATE_ESTIMATOR_MSGS::SLIP_DETECTION"))

(in-package "ROS")
;;//! \htmlinclude slip_detection.msg.html


(defclass state_estimator_msgs::slip_detection
  :super ros::object
  :slots (_legs_slipping _slip_lf _slip_rf _slip_lh _slip_rh _slippage ))

(defmethod state_estimator_msgs::slip_detection
  (:init
   (&key
    ((:legs_slipping __legs_slipping) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:slip_lf __slip_lf) nil)
    ((:slip_rf __slip_rf) nil)
    ((:slip_lh __slip_lh) nil)
    ((:slip_rh __slip_rh) nil)
    ((:slippage __slippage) nil)
    )
   (send-super :init)
   (setq _legs_slipping __legs_slipping)
   (setq _slip_lf __slip_lf)
   (setq _slip_rf __slip_rf)
   (setq _slip_lh __slip_lh)
   (setq _slip_rh __slip_rh)
   (setq _slippage __slippage)
   self)
  (:legs_slipping
   (&optional __legs_slipping)
   (if __legs_slipping (setq _legs_slipping __legs_slipping)) _legs_slipping)
  (:slip_lf
   (&optional (__slip_lf :null))
   (if (not (eq __slip_lf :null)) (setq _slip_lf __slip_lf)) _slip_lf)
  (:slip_rf
   (&optional (__slip_rf :null))
   (if (not (eq __slip_rf :null)) (setq _slip_rf __slip_rf)) _slip_rf)
  (:slip_lh
   (&optional (__slip_lh :null))
   (if (not (eq __slip_lh :null)) (setq _slip_lh __slip_lh)) _slip_lh)
  (:slip_rh
   (&optional (__slip_rh :null))
   (if (not (eq __slip_rh :null)) (setq _slip_rh __slip_rh)) _slip_rh)
  (:slippage
   (&optional (__slippage :null))
   (if (not (eq __slippage :null)) (setq _slippage __slippage)) _slippage)
  (:serialization-length
   ()
   (+
    ;; float64[4] _legs_slipping
    (* 8    4)
    ;; bool _slip_lf
    1
    ;; bool _slip_rf
    1
    ;; bool _slip_lh
    1
    ;; bool _slip_rh
    1
    ;; bool _slippage
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[4] _legs_slipping
     (dotimes (i 4)
       (sys::poke (elt _legs_slipping i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _slip_lf
       (if _slip_lf (write-byte -1 s) (write-byte 0 s))
     ;; bool _slip_rf
       (if _slip_rf (write-byte -1 s) (write-byte 0 s))
     ;; bool _slip_lh
       (if _slip_lh (write-byte -1 s) (write-byte 0 s))
     ;; bool _slip_rh
       (if _slip_rh (write-byte -1 s) (write-byte 0 s))
     ;; bool _slippage
       (if _slippage (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[4] _legs_slipping
   (dotimes (i (length _legs_slipping))
     (setf (elt _legs_slipping i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; bool _slip_lf
     (setq _slip_lf (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _slip_rf
     (setq _slip_rf (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _slip_lh
     (setq _slip_lh (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _slip_rh
     (setq _slip_rh (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _slippage
     (setq _slippage (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get state_estimator_msgs::slip_detection :md5sum-) "87252336384194759607e2e036859110")
(setf (get state_estimator_msgs::slip_detection :datatype-) "state_estimator_msgs/slip_detection")
(setf (get state_estimator_msgs::slip_detection :definition-)
      "float64[4] legs_slipping
bool slip_lf
bool slip_rf
bool slip_lh
bool slip_rh
bool slippage
")



(provide :state_estimator_msgs/slip_detection "87252336384194759607e2e036859110")


