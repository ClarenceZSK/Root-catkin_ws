; Auto-generated. Do not edit!


(cl:in-package sar_localization-msg)


;//! \htmlinclude Csi.msg.html

(cl:defclass <Csi> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (cos_value
    :reader cos_value
    :initarg :cos_value
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64)))
)

(cl:defclass Csi (<Csi>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Csi>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Csi)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sar_localization-msg:<Csi> is deprecated: use sar_localization-msg:Csi instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Csi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sar_localization-msg:header-val is deprecated.  Use sar_localization-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'cos_value-val :lambda-list '(m))
(cl:defmethod cos_value-val ((m <Csi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sar_localization-msg:cos_value-val is deprecated.  Use sar_localization-msg:cos_value instead.")
  (cos_value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Csi>) ostream)
  "Serializes a message object of type '<Csi>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cos_value) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Csi>) istream)
  "Deserializes a message object of type '<Csi>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cos_value) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Csi>)))
  "Returns string type for a message object of type '<Csi>"
  "sar_localization/Csi")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Csi)))
  "Returns string type for a message object of type 'Csi"
  "sar_localization/Csi")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Csi>)))
  "Returns md5sum for a message object of type '<Csi>"
  "243b6b66f6697b18c3d1b19c55371a5a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Csi)))
  "Returns md5sum for a message object of type 'Csi"
  "243b6b66f6697b18c3d1b19c55371a5a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Csi>)))
  "Returns full string definition for message of type '<Csi>"
  (cl:format cl:nil "Header header~%std_msgs/Float64 cos_value~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Csi)))
  "Returns full string definition for message of type 'Csi"
  (cl:format cl:nil "Header header~%std_msgs/Float64 cos_value~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Csi>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cos_value))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Csi>))
  "Converts a ROS message object to a list"
  (cl:list 'Csi
    (cl:cons ':header (header msg))
    (cl:cons ':cos_value (cos_value msg))
))
