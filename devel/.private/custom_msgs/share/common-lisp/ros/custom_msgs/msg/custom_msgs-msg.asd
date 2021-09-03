
(cl:in-package :asdf)

(defsystem "custom_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "motor_pwm" :depends-on ("_package_motor_pwm"))
    (:file "_package_motor_pwm" :depends-on ("_package"))
  ))