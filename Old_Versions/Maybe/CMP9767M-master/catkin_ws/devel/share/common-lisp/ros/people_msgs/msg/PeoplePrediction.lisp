; Auto-generated. Do not edit!


(cl:in-package people_msgs-msg)


;//! \htmlinclude PeoplePrediction.msg.html

(cl:defclass <PeoplePrediction> (roslisp-msg-protocol:ros-message)
  ((predicted_people
    :reader predicted_people
    :initarg :predicted_people
    :type (cl:vector people_msgs-msg:People)
   :initform (cl:make-array 0 :element-type 'people_msgs-msg:People :initial-element (cl:make-instance 'people_msgs-msg:People))))
)

(cl:defclass PeoplePrediction (<PeoplePrediction>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PeoplePrediction>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PeoplePrediction)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name people_msgs-msg:<PeoplePrediction> is deprecated: use people_msgs-msg:PeoplePrediction instead.")))

(cl:ensure-generic-function 'predicted_people-val :lambda-list '(m))
(cl:defmethod predicted_people-val ((m <PeoplePrediction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader people_msgs-msg:predicted_people-val is deprecated.  Use people_msgs-msg:predicted_people instead.")
  (predicted_people m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PeoplePrediction>) ostream)
  "Serializes a message object of type '<PeoplePrediction>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'predicted_people))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'predicted_people))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PeoplePrediction>) istream)
  "Deserializes a message object of type '<PeoplePrediction>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'predicted_people) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'predicted_people)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'people_msgs-msg:People))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PeoplePrediction>)))
  "Returns string type for a message object of type '<PeoplePrediction>"
  "people_msgs/PeoplePrediction")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PeoplePrediction)))
  "Returns string type for a message object of type 'PeoplePrediction"
  "people_msgs/PeoplePrediction")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PeoplePrediction>)))
  "Returns md5sum for a message object of type '<PeoplePrediction>"
  "651f35473deabee93f26f3b7e8e8373d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PeoplePrediction)))
  "Returns md5sum for a message object of type 'PeoplePrediction"
  "651f35473deabee93f26f3b7e8e8373d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PeoplePrediction>)))
  "Returns full string definition for message of type '<PeoplePrediction>"
  (cl:format cl:nil "people_msgs/People[] predicted_people~%~%================================================================================~%MSG: people_msgs/People~%Header header~%people_msgs/Person[] people~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: people_msgs/Person~%string              name~%geometry_msgs/Point position~%geometry_msgs/Point velocity~%float64             reliability~%string[]            tagnames~%string[]            tags~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PeoplePrediction)))
  "Returns full string definition for message of type 'PeoplePrediction"
  (cl:format cl:nil "people_msgs/People[] predicted_people~%~%================================================================================~%MSG: people_msgs/People~%Header header~%people_msgs/Person[] people~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: people_msgs/Person~%string              name~%geometry_msgs/Point position~%geometry_msgs/Point velocity~%float64             reliability~%string[]            tagnames~%string[]            tags~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PeoplePrediction>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'predicted_people) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PeoplePrediction>))
  "Converts a ROS message object to a list"
  (cl:list 'PeoplePrediction
    (cl:cons ':predicted_people (predicted_people msg))
))
