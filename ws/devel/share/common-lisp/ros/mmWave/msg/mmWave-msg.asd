
(cl:in-package :asdf)

(defsystem "mmWave-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "data_frame" :depends-on ("_package_data_frame"))
    (:file "_package_data_frame" :depends-on ("_package"))
  ))