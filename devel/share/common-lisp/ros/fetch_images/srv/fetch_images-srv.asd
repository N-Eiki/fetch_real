
(cl:in-package :asdf)

(defsystem "fetch_images-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "GetImages" :depends-on ("_package_GetImages"))
    (:file "_package_GetImages" :depends-on ("_package"))
  ))