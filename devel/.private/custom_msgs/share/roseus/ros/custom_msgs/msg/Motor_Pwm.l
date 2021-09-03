;; Auto-generated. Do not edit!


(when (boundp 'custom_msgs::Motor_Pwm)
  (if (not (find-package "CUSTOM_MSGS"))
    (make-package "CUSTOM_MSGS"))
  (shadow 'Motor_Pwm (find-package "CUSTOM_MSGS")))
(unless (find-package "CUSTOM_MSGS::MOTOR_PWM")
  (make-package "CUSTOM_MSGS::MOTOR_PWM"))

(in-package "ROS")
;;//! \htmlinclude Motor_Pwm.msg.html


(defclass custom_msgs::Motor_Pwm
  :super ros::object
  :slots (_L _R ))

(defmethod custom_msgs::Motor_Pwm
  (:init
   (&key
    ((:L __L) 0)
    ((:R __R) 0)
    )
   (send-super :init)
   (setq _L (round __L))
   (setq _R (round __R))
   self)
  (:L
   (&optional __L)
   (if __L (setq _L __L)) _L)
  (:R
   (&optional __R)
   (if __R (setq _R __R)) _R)
  (:serialization-length
   ()
   (+
    ;; int64 _L
    8
    ;; int64 _R
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _L
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _L (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _L) (= (length (_L . bv)) 2)) ;; bignum
              (write-long (ash (elt (_L . bv) 0) 0) s)
              (write-long (ash (elt (_L . bv) 1) -1) s))
             ((and (class _L) (= (length (_L . bv)) 1)) ;; big1
              (write-long (elt (_L . bv) 0) s)
              (write-long (if (>= _L 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _L s)(write-long (if (>= _L 0) 0 #xffffffff) s)))
     ;; int64 _R
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _R (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _R) (= (length (_R . bv)) 2)) ;; bignum
              (write-long (ash (elt (_R . bv) 0) 0) s)
              (write-long (ash (elt (_R . bv) 1) -1) s))
             ((and (class _R) (= (length (_R . bv)) 1)) ;; big1
              (write-long (elt (_R . bv) 0) s)
              (write-long (if (>= _R 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _R s)(write-long (if (>= _R 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _L
#+(or :alpha :irix6 :x86_64)
      (setf _L (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _L (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _R
#+(or :alpha :irix6 :x86_64)
      (setf _R (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _R (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get custom_msgs::Motor_Pwm :md5sum-) "4184f594ee6fa4706c2c2eca40be03fe")
(setf (get custom_msgs::Motor_Pwm :datatype-) "custom_msgs/Motor_Pwm")
(setf (get custom_msgs::Motor_Pwm :definition-)
      "int64 L
int64 R

")



(provide :custom_msgs/Motor_Pwm "4184f594ee6fa4706c2c2eca40be03fe")


