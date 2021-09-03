; Auto-generated. Do not edit!


(cl:in-package custom_msgs-msg)


;//! \htmlinclude Motor_Pwm.msg.html

(cl:defclass <Motor_Pwm> (roslisp-msg-protocol:ros-message)
  ((L
    :reader L
    :initarg :L
    :type cl:integer
    :initform 0)
   (R
    :reader R
    :initarg :R
    :type cl:integer
    :initform 0))
)

(cl:defclass Motor_Pwm (<Motor_Pwm>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Motor_Pwm>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Motor_Pwm)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs-msg:<Motor_Pwm> is deprecated: use custom_msgs-msg:Motor_Pwm instead.")))

(cl:ensure-generic-function 'L-val :lambda-list '(m))
(cl:defmethod L-val ((m <Motor_Pwm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:L-val is deprecated.  Use custom_msgs-msg:L instead.")
  (L m))

(cl:ensure-generic-function 'R-val :lambda-list '(m))
(cl:defmethod R-val ((m <Motor_Pwm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:R-val is deprecated.  Use custom_msgs-msg:R instead.")
  (R m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Motor_Pwm>) ostream)
  "Serializes a message object of type '<Motor_Pwm>"
  (cl:let* ((signed (cl:slot-value msg 'L)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'R)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Motor_Pwm>) istream)
  "Deserializes a message object of type '<Motor_Pwm>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'L) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'R) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Motor_Pwm>)))
  "Returns string type for a message object of type '<Motor_Pwm>"
  "custom_msgs/Motor_Pwm")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Motor_Pwm)))
  "Returns string type for a message object of type 'Motor_Pwm"
  "custom_msgs/Motor_Pwm")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Motor_Pwm>)))
  "Returns md5sum for a message object of type '<Motor_Pwm>"
  "4184f594ee6fa4706c2c2eca40be03fe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Motor_Pwm)))
  "Returns md5sum for a message object of type 'Motor_Pwm"
  "4184f594ee6fa4706c2c2eca40be03fe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Motor_Pwm>)))
  "Returns full string definition for message of type '<Motor_Pwm>"
  (cl:format cl:nil "int64 L~%int64 R~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Motor_Pwm)))
  "Returns full string definition for message of type 'Motor_Pwm"
  (cl:format cl:nil "int64 L~%int64 R~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Motor_Pwm>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Motor_Pwm>))
  "Converts a ROS message object to a list"
  (cl:list 'Motor_Pwm
    (cl:cons ':L (L msg))
    (cl:cons ':R (R msg))
))
