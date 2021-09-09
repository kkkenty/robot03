
(cl:in-package :asdf)

(defsystem "msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Motor" :depends-on ("_package_Motor"))
    (:file "_package_Motor" :depends-on ("_package"))
  ))