; Auto-generated. Do not edit!


(cl:in-package sar_localization-msg)


;//! \htmlinclude Csi.msg.html

(cl:defclass <Csi> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (Ntx
    :reader Ntx
    :initarg :Ntx
    :type cl:fixnum
    :initform 0)
   (csi1_real
    :reader csi1_real
    :initarg :csi1_real
    :type std_msgs-msg:Float64MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float64MultiArray))
   (csi1_image
    :reader csi1_image
    :initarg :csi1_image
    :type std_msgs-msg:Float64MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float64MultiArray))
   (csi2_real
    :reader csi2_real
    :initarg :csi2_real
    :type std_msgs-msg:Float64MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float64MultiArray))
   (csi2_image
    :reader csi2_image
    :initarg :csi2_image
    :type std_msgs-msg:Float64MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float64MultiArray))
   (check_csi
    :reader check_csi
    :initarg :check_csi
    :type cl:boolean
    :initform cl:nil))
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

(cl:ensure-generic-function 'Ntx-val :lambda-list '(m))
(cl:defmethod Ntx-val ((m <Csi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sar_localization-msg:Ntx-val is deprecated.  Use sar_localization-msg:Ntx instead.")
  (Ntx m))

(cl:ensure-generic-function 'csi1_real-val :lambda-list '(m))
(cl:defmethod csi1_real-val ((m <Csi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sar_localization-msg:csi1_real-val is deprecated.  Use sar_localization-msg:csi1_real instead.")
  (csi1_real m))

(cl:ensure-generic-function 'csi1_image-val :lambda-list '(m))
(cl:defmethod csi1_image-val ((m <Csi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sar_localization-msg:csi1_image-val is deprecated.  Use sar_localization-msg:csi1_image instead.")
  (csi1_image m))

(cl:ensure-generic-function 'csi2_real-val :lambda-list '(m))
(cl:defmethod csi2_real-val ((m <Csi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sar_localization-msg:csi2_real-val is deprecated.  Use sar_localization-msg:csi2_real instead.")
  (csi2_real m))

(cl:ensure-generic-function 'csi2_image-val :lambda-list '(m))
(cl:defmethod csi2_image-val ((m <Csi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sar_localization-msg:csi2_image-val is deprecated.  Use sar_localization-msg:csi2_image instead.")
  (csi2_image m))

(cl:ensure-generic-function 'check_csi-val :lambda-list '(m))
(cl:defmethod check_csi-val ((m <Csi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sar_localization-msg:check_csi-val is deprecated.  Use sar_localization-msg:check_csi instead.")
  (check_csi m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Csi>) ostream)
  "Serializes a message object of type '<Csi>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'Ntx)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'csi1_real) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'csi1_image) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'csi2_real) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'csi2_image) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'check_csi) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Csi>) istream)
  "Deserializes a message object of type '<Csi>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'Ntx)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'csi1_real) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'csi1_image) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'csi2_real) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'csi2_image) istream)
    (cl:setf (cl:slot-value msg 'check_csi) (cl:not (cl:zerop (cl:read-byte istream))))
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
  "d4390fb55572f1efc2a1c6b8190d5e0d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Csi)))
  "Returns md5sum for a message object of type 'Csi"
  "d4390fb55572f1efc2a1c6b8190d5e0d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Csi>)))
  "Returns full string definition for message of type '<Csi>"
  (cl:format cl:nil "Header header~%uint8 Ntx~%std_msgs/Float64MultiArray csi1_real~%std_msgs/Float64MultiArray csi1_image~%std_msgs/Float64MultiArray csi2_real~%std_msgs/Float64MultiArray csi2_image~%bool check_csi~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding bytes at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Csi)))
  "Returns full string definition for message of type 'Csi"
  (cl:format cl:nil "Header header~%uint8 Ntx~%std_msgs/Float64MultiArray csi1_real~%std_msgs/Float64MultiArray csi1_image~%std_msgs/Float64MultiArray csi2_real~%std_msgs/Float64MultiArray csi2_image~%bool check_csi~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding bytes at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Csi>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'csi1_real))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'csi1_image))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'csi2_real))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'csi2_image))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Csi>))
  "Converts a ROS message object to a list"
  (cl:list 'Csi
    (cl:cons ':header (header msg))
    (cl:cons ':Ntx (Ntx msg))
    (cl:cons ':csi1_real (csi1_real msg))
    (cl:cons ':csi1_image (csi1_image msg))
    (cl:cons ':csi2_real (csi2_real msg))
    (cl:cons ':csi2_image (csi2_image msg))
    (cl:cons ':check_csi (check_csi msg))
))
