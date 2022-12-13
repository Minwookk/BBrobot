
(cl:in-package :asdf)

(defsystem "mmdetection_ros-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :vision_msgs-msg
)
  :components ((:file "_package")
    (:file "mmdetSrv" :depends-on ("_package_mmdetSrv"))
    (:file "_package_mmdetSrv" :depends-on ("_package"))
  ))