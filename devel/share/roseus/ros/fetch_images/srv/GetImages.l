;; Auto-generated. Do not edit!


(when (boundp 'fetch_images::GetImages)
  (if (not (find-package "FETCH_IMAGES"))
    (make-package "FETCH_IMAGES"))
  (shadow 'GetImages (find-package "FETCH_IMAGES")))
(unless (find-package "FETCH_IMAGES::GETIMAGES")
  (make-package "FETCH_IMAGES::GETIMAGES"))
(unless (find-package "FETCH_IMAGES::GETIMAGESREQUEST")
  (make-package "FETCH_IMAGES::GETIMAGESREQUEST"))
(unless (find-package "FETCH_IMAGES::GETIMAGESRESPONSE")
  (make-package "FETCH_IMAGES::GETIMAGESRESPONSE"))

(in-package "ROS")



(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass fetch_images::GetImagesRequest
  :super ros::object
  :slots ())

(defmethod fetch_images::GetImagesRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass fetch_images::GetImagesResponse
  :super ros::object
  :slots (_rgb_image _depth_image ))

(defmethod fetch_images::GetImagesResponse
  (:init
   (&key
    ((:rgb_image __rgb_image) (instance sensor_msgs::Image :init))
    ((:depth_image __depth_image) (instance sensor_msgs::Image :init))
    )
   (send-super :init)
   (setq _rgb_image __rgb_image)
   (setq _depth_image __depth_image)
   self)
  (:rgb_image
   (&rest __rgb_image)
   (if (keywordp (car __rgb_image))
       (send* _rgb_image __rgb_image)
     (progn
       (if __rgb_image (setq _rgb_image (car __rgb_image)))
       _rgb_image)))
  (:depth_image
   (&rest __depth_image)
   (if (keywordp (car __depth_image))
       (send* _depth_image __depth_image)
     (progn
       (if __depth_image (setq _depth_image (car __depth_image)))
       _depth_image)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/Image _rgb_image
    (send _rgb_image :serialization-length)
    ;; sensor_msgs/Image _depth_image
    (send _depth_image :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/Image _rgb_image
       (send _rgb_image :serialize s)
     ;; sensor_msgs/Image _depth_image
       (send _depth_image :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/Image _rgb_image
     (send _rgb_image :deserialize buf ptr-) (incf ptr- (send _rgb_image :serialization-length))
   ;; sensor_msgs/Image _depth_image
     (send _depth_image :deserialize buf ptr-) (incf ptr- (send _depth_image :serialization-length))
   ;;
   self)
  )

(defclass fetch_images::GetImages
  :super ros::object
  :slots ())

(setf (get fetch_images::GetImages :md5sum-) "33cb5836216cc2f10faa186b171cbfdc")
(setf (get fetch_images::GetImages :datatype-) "fetch_images/GetImages")
(setf (get fetch_images::GetImages :request) fetch_images::GetImagesRequest)
(setf (get fetch_images::GetImages :response) fetch_images::GetImagesResponse)

(defmethod fetch_images::GetImagesRequest
  (:response () (instance fetch_images::GetImagesResponse :init)))

(setf (get fetch_images::GetImagesRequest :md5sum-) "33cb5836216cc2f10faa186b171cbfdc")
(setf (get fetch_images::GetImagesRequest :datatype-) "fetch_images/GetImagesRequest")
(setf (get fetch_images::GetImagesRequest :definition-)
      "---
sensor_msgs/Image rgb_image
sensor_msgs/Image depth_image


================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

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
string frame_id
")

(setf (get fetch_images::GetImagesResponse :md5sum-) "33cb5836216cc2f10faa186b171cbfdc")
(setf (get fetch_images::GetImagesResponse :datatype-) "fetch_images/GetImagesResponse")
(setf (get fetch_images::GetImagesResponse :definition-)
      "---
sensor_msgs/Image rgb_image
sensor_msgs/Image depth_image


================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

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
string frame_id
")



(provide :fetch_images/GetImages "33cb5836216cc2f10faa186b171cbfdc")


