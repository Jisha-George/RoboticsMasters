;; Auto-generated. Do not edit!


(when (boundp 'people_msgs::PeoplePrediction)
  (if (not (find-package "PEOPLE_MSGS"))
    (make-package "PEOPLE_MSGS"))
  (shadow 'PeoplePrediction (find-package "PEOPLE_MSGS")))
(unless (find-package "PEOPLE_MSGS::PEOPLEPREDICTION")
  (make-package "PEOPLE_MSGS::PEOPLEPREDICTION"))

(in-package "ROS")
;;//! \htmlinclude PeoplePrediction.msg.html


(defclass people_msgs::PeoplePrediction
  :super ros::object
  :slots (_predicted_people ))

(defmethod people_msgs::PeoplePrediction
  (:init
   (&key
    ((:predicted_people __predicted_people) (let (r) (dotimes (i 0) (push (instance people_msgs::People :init) r)) r))
    )
   (send-super :init)
   (setq _predicted_people __predicted_people)
   self)
  (:predicted_people
   (&rest __predicted_people)
   (if (keywordp (car __predicted_people))
       (send* _predicted_people __predicted_people)
     (progn
       (if __predicted_people (setq _predicted_people (car __predicted_people)))
       _predicted_people)))
  (:serialization-length
   ()
   (+
    ;; people_msgs/People[] _predicted_people
    (apply #'+ (send-all _predicted_people :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; people_msgs/People[] _predicted_people
     (write-long (length _predicted_people) s)
     (dolist (elem _predicted_people)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; people_msgs/People[] _predicted_people
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _predicted_people (let (r) (dotimes (i n) (push (instance people_msgs::People :init) r)) r))
     (dolist (elem- _predicted_people)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get people_msgs::PeoplePrediction :md5sum-) "651f35473deabee93f26f3b7e8e8373d")
(setf (get people_msgs::PeoplePrediction :datatype-) "people_msgs/PeoplePrediction")
(setf (get people_msgs::PeoplePrediction :definition-)
      "people_msgs/People[] predicted_people

================================================================================
MSG: people_msgs/People
Header header
people_msgs/Person[] people

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: people_msgs/Person
string              name
geometry_msgs/Point position
geometry_msgs/Point velocity
float64             reliability
string[]            tagnames
string[]            tags


================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :people_msgs/PeoplePrediction "651f35473deabee93f26f3b7e8e8373d")


