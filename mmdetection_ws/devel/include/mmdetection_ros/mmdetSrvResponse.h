// Generated by gencpp from file mmdetection_ros/mmdetSrvResponse.msg
// DO NOT EDIT!


#ifndef MMDETECTION_ROS_MESSAGE_MMDETSRVRESPONSE_H
#define MMDETECTION_ROS_MESSAGE_MMDETSRVRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <vision_msgs/Detection2DArray.h>

namespace mmdetection_ros
{
template <class ContainerAllocator>
struct mmdetSrvResponse_
{
  typedef mmdetSrvResponse_<ContainerAllocator> Type;

  mmdetSrvResponse_()
    : results()  {
    }
  mmdetSrvResponse_(const ContainerAllocator& _alloc)
    : results(_alloc)  {
  (void)_alloc;
    }



   typedef  ::vision_msgs::Detection2DArray_<ContainerAllocator>  _results_type;
  _results_type results;





  typedef boost::shared_ptr< ::mmdetection_ros::mmdetSrvResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mmdetection_ros::mmdetSrvResponse_<ContainerAllocator> const> ConstPtr;

}; // struct mmdetSrvResponse_

typedef ::mmdetection_ros::mmdetSrvResponse_<std::allocator<void> > mmdetSrvResponse;

typedef boost::shared_ptr< ::mmdetection_ros::mmdetSrvResponse > mmdetSrvResponsePtr;
typedef boost::shared_ptr< ::mmdetection_ros::mmdetSrvResponse const> mmdetSrvResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mmdetection_ros::mmdetSrvResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mmdetection_ros::mmdetSrvResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mmdetection_ros::mmdetSrvResponse_<ContainerAllocator1> & lhs, const ::mmdetection_ros::mmdetSrvResponse_<ContainerAllocator2> & rhs)
{
  return lhs.results == rhs.results;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mmdetection_ros::mmdetSrvResponse_<ContainerAllocator1> & lhs, const ::mmdetection_ros::mmdetSrvResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mmdetection_ros

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mmdetection_ros::mmdetSrvResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mmdetection_ros::mmdetSrvResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mmdetection_ros::mmdetSrvResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mmdetection_ros::mmdetSrvResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mmdetection_ros::mmdetSrvResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mmdetection_ros::mmdetSrvResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mmdetection_ros::mmdetSrvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5cf8ff1e509acd20d36134d832fd1ba9";
  }

  static const char* value(const ::mmdetection_ros::mmdetSrvResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5cf8ff1e509acd20ULL;
  static const uint64_t static_value2 = 0xd36134d832fd1ba9ULL;
};

template<class ContainerAllocator>
struct DataType< ::mmdetection_ros::mmdetSrvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mmdetection_ros/mmdetSrvResponse";
  }

  static const char* value(const ::mmdetection_ros::mmdetSrvResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mmdetection_ros::mmdetSrvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vision_msgs/Detection2DArray results\n"
"\n"
"\n"
"================================================================================\n"
"MSG: vision_msgs/Detection2DArray\n"
"# A list of 2D detections, for a multi-object 2D detector.\n"
"\n"
"Header header\n"
"\n"
"# A list of the detected proposals. A multi-proposal detector might generate\n"
"#   this list with many candidate detections generated from a single input.\n"
"Detection2D[] detections\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: vision_msgs/Detection2D\n"
"# Defines a 2D detection result.\n"
"#\n"
"# This is similar to a 2D classification, but includes position information,\n"
"#   allowing a classification result for a specific crop or image point to\n"
"#   to be located in the larger image.\n"
"\n"
"Header header\n"
"\n"
"# Class probabilities\n"
"ObjectHypothesisWithPose[] results\n"
"\n"
"# 2D bounding box surrounding the object.\n"
"BoundingBox2D bbox\n"
"\n"
"# The 2D data that generated these results (i.e. region proposal cropped out of\n"
"#   the image). Not required for all use cases, so it may be empty.\n"
"sensor_msgs/Image source_img\n"
"\n"
"================================================================================\n"
"MSG: vision_msgs/ObjectHypothesisWithPose\n"
"# An object hypothesis that contains position information.\n"
"\n"
"# The unique numeric ID of object detected. To get additional information about\n"
"#   this ID, such as its human-readable name, listeners should perform a lookup\n"
"#   in a metadata database. See vision_msgs/VisionInfo.msg for more detail.\n"
"int64 id\n"
"\n"
"# The probability or confidence value of the detected object. By convention,\n"
"#   this value should lie in the range [0-1].\n"
"float64 score\n"
"\n"
"# The 6D pose of the object hypothesis. This pose should be\n"
"#   defined as the pose of some fixed reference point on the object, such a\n"
"#   the geometric center of the bounding box or the center of mass of the\n"
"#   object.\n"
"# Note that this pose is not stamped; frame information can be defined by\n"
"#   parent messages.\n"
"# Also note that different classes predicted for the same input data may have\n"
"#   different predicted 6D poses.\n"
"geometry_msgs/PoseWithCovariance pose\n"
"================================================================================\n"
"MSG: geometry_msgs/PoseWithCovariance\n"
"# This represents a pose in free space with uncertainty.\n"
"\n"
"Pose pose\n"
"\n"
"# Row-major representation of the 6x6 covariance matrix\n"
"# The orientation parameters use a fixed-axis representation.\n"
"# In order, the parameters are:\n"
"# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n"
"float64[36] covariance\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
"\n"
"================================================================================\n"
"MSG: vision_msgs/BoundingBox2D\n"
"# A 2D bounding box that can be rotated about its center.\n"
"# All dimensions are in pixels, but represented using floating-point\n"
"#   values to allow sub-pixel precision. If an exact pixel crop is required\n"
"#   for a rotated bounding box, it can be calculated using Bresenham's line\n"
"#   algorithm.\n"
"\n"
"# The 2D position (in pixels) and orientation of the bounding box center.\n"
"geometry_msgs/Pose2D center\n"
"\n"
"# The size (in pixels) of the bounding box surrounding the object relative\n"
"#   to the pose of its center.\n"
"float64 size_x\n"
"float64 size_y\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose2D\n"
"# Deprecated\n"
"# Please use the full 3D pose.\n"
"\n"
"# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.\n"
"\n"
"# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.\n"
"\n"
"\n"
"# This expresses a position and orientation on a 2D manifold.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 theta\n"
"\n"
"================================================================================\n"
"MSG: sensor_msgs/Image\n"
"# This message contains an uncompressed image\n"
"# (0, 0) is at top-left corner of image\n"
"#\n"
"\n"
"Header header        # Header timestamp should be acquisition time of image\n"
"                     # Header frame_id should be optical frame of camera\n"
"                     # origin of frame should be optical center of camera\n"
"                     # +x should point to the right in the image\n"
"                     # +y should point down in the image\n"
"                     # +z should point into to plane of the image\n"
"                     # If the frame_id here and the frame_id of the CameraInfo\n"
"                     # message associated with the image conflict\n"
"                     # the behavior is undefined\n"
"\n"
"uint32 height         # image height, that is, number of rows\n"
"uint32 width          # image width, that is, number of columns\n"
"\n"
"# The legal values for encoding are in file src/image_encodings.cpp\n"
"# If you want to standardize a new string format, join\n"
"# ros-users@lists.sourceforge.net and send an email proposing a new encoding.\n"
"\n"
"string encoding       # Encoding of pixels -- channel meaning, ordering, size\n"
"                      # taken from the list of strings in include/sensor_msgs/image_encodings.h\n"
"\n"
"uint8 is_bigendian    # is this data bigendian?\n"
"uint32 step           # Full row length in bytes\n"
"uint8[] data          # actual matrix data, size is (step * rows)\n"
;
  }

  static const char* value(const ::mmdetection_ros::mmdetSrvResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mmdetection_ros::mmdetSrvResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.results);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct mmdetSrvResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mmdetection_ros::mmdetSrvResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mmdetection_ros::mmdetSrvResponse_<ContainerAllocator>& v)
  {
    s << indent << "results: ";
    s << std::endl;
    Printer< ::vision_msgs::Detection2DArray_<ContainerAllocator> >::stream(s, indent + "  ", v.results);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MMDETECTION_ROS_MESSAGE_MMDETSRVRESPONSE_H
