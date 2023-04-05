; Auto-generated. Do not edit!


(cl:in-package fetch_images-srv)


;//! \htmlinclude GetImages-request.msg.html

(cl:defclass <GetImages-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetImages-request (<GetImages-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetImages-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetImages-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fetch_images-srv:<GetImages-request> is deprecated: use fetch_images-srv:GetImages-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetImages-request>) ostream)
  "Serializes a message object of type '<GetImages-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetImages-request>) istream)
  "Deserializes a message object of type '<GetImages-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetImages-request>)))
  "Returns string type for a service object of type '<GetImages-request>"
  "fetch_images/GetImagesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetImages-request)))
  "Returns string type for a service object of type 'GetImages-request"
  "fetch_images/GetImagesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetImages-request>)))
  "Returns md5sum for a message object of type '<GetImages-request>"
  "33cb5836216cc2f10faa186b171cbfdc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetImages-request)))
  "Returns md5sum for a message object of type 'GetImages-request"
  "33cb5836216cc2f10faa186b171cbfdc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetImages-request>)))
  "Returns full string definition for message of type '<GetImages-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetImages-request)))
  "Returns full string definition for message of type 'GetImages-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetImages-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetImages-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetImages-request
))
;//! \htmlinclude GetImages-response.msg.html

(cl:defclass <GetImages-response> (roslisp-msg-protocol:ros-message)
  ((rgb_image
    :reader rgb_image
    :initarg :rgb_image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (depth_image
    :reader depth_image
    :initarg :depth_image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass GetImages-response (<GetImages-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetImages-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetImages-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fetch_images-srv:<GetImages-response> is deprecated: use fetch_images-srv:GetImages-response instead.")))

(cl:ensure-generic-function 'rgb_image-val :lambda-list '(m))
(cl:defmethod rgb_image-val ((m <GetImages-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fetch_images-srv:rgb_image-val is deprecated.  Use fetch_images-srv:rgb_image instead.")
  (rgb_image m))

(cl:ensure-generic-function 'depth_image-val :lambda-list '(m))
(cl:defmethod depth_image-val ((m <GetImages-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fetch_images-srv:depth_image-val is deprecated.  Use fetch_images-srv:depth_image instead.")
  (depth_image m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetImages-response>) ostream)
  "Serializes a message object of type '<GetImages-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'rgb_image) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'depth_image) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetImages-response>) istream)
  "Deserializes a message object of type '<GetImages-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'rgb_image) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'depth_image) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetImages-response>)))
  "Returns string type for a service object of type '<GetImages-response>"
  "fetch_images/GetImagesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetImages-response)))
  "Returns string type for a service object of type 'GetImages-response"
  "fetch_images/GetImagesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetImages-response>)))
  "Returns md5sum for a message object of type '<GetImages-response>"
  "33cb5836216cc2f10faa186b171cbfdc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetImages-response)))
  "Returns md5sum for a message object of type 'GetImages-response"
  "33cb5836216cc2f10faa186b171cbfdc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetImages-response>)))
  "Returns full string definition for message of type '<GetImages-response>"
  (cl:format cl:nil "sensor_msgs/Image rgb_image~%sensor_msgs/Image depth_image~%~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetImages-response)))
  "Returns full string definition for message of type 'GetImages-response"
  (cl:format cl:nil "sensor_msgs/Image rgb_image~%sensor_msgs/Image depth_image~%~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetImages-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'rgb_image))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'depth_image))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetImages-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetImages-response
    (cl:cons ':rgb_image (rgb_image msg))
    (cl:cons ':depth_image (depth_image msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetImages)))
  'GetImages-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetImages)))
  'GetImages-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetImages)))
  "Returns string type for a service object of type '<GetImages>"
  "fetch_images/GetImages")