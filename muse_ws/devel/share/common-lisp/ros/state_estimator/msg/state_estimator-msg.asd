
(cl:in-package :asdf)

(defsystem "state_estimator-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ContactGazebo" :depends-on ("_package_ContactGazebo"))
    (:file "_package_ContactGazebo" :depends-on ("_package"))
    (:file "ContactGazebo" :depends-on ("_package_ContactGazebo"))
    (:file "_package_ContactGazebo" :depends-on ("_package"))
    (:file "RollPitch" :depends-on ("_package_RollPitch"))
    (:file "_package_RollPitch" :depends-on ("_package"))
    (:file "RollPitch" :depends-on ("_package_RollPitch"))
    (:file "_package_RollPitch" :depends-on ("_package"))
    (:file "RollPitchStamped" :depends-on ("_package_RollPitchStamped"))
    (:file "_package_RollPitchStamped" :depends-on ("_package"))
    (:file "RollPitchStamped" :depends-on ("_package_RollPitchStamped"))
    (:file "_package_RollPitchStamped" :depends-on ("_package"))
  ))