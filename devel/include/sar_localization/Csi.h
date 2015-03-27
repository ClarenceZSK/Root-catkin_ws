// Generated by gencpp from file sar_localization/Csi.msg
// DO NOT EDIT!


#ifndef SAR_LOCALIZATION_MESSAGE_CSI_H
#define SAR_LOCALIZATION_MESSAGE_CSI_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64MultiArray.h>

namespace sar_localization
{
template <class ContainerAllocator>
struct Csi_
{
  typedef Csi_<ContainerAllocator> Type;

  Csi_()
    : header()
    , Ntx(0)
    , csi1_real()
    , csi1_image()
    , csi2_real()
    , csi2_image()
    , check_csi(false)  {
    }
  Csi_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , Ntx(0)
    , csi1_real(_alloc)
    , csi1_image(_alloc)
    , csi2_real(_alloc)
    , csi2_image(_alloc)
    , check_csi(false)  {
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _Ntx_type;
  _Ntx_type Ntx;

   typedef  ::std_msgs::Float64MultiArray_<ContainerAllocator>  _csi1_real_type;
  _csi1_real_type csi1_real;

   typedef  ::std_msgs::Float64MultiArray_<ContainerAllocator>  _csi1_image_type;
  _csi1_image_type csi1_image;

   typedef  ::std_msgs::Float64MultiArray_<ContainerAllocator>  _csi2_real_type;
  _csi2_real_type csi2_real;

   typedef  ::std_msgs::Float64MultiArray_<ContainerAllocator>  _csi2_image_type;
  _csi2_image_type csi2_image;

   typedef uint8_t _check_csi_type;
  _check_csi_type check_csi;




  typedef boost::shared_ptr< ::sar_localization::Csi_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::sar_localization::Csi_<ContainerAllocator> const> ConstPtr;

}; // struct Csi_

typedef ::sar_localization::Csi_<std::allocator<void> > Csi;

typedef boost::shared_ptr< ::sar_localization::Csi > CsiPtr;
typedef boost::shared_ptr< ::sar_localization::Csi const> CsiConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sar_localization::Csi_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::sar_localization::Csi_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace sar_localization

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'sar_localization': ['/root/catkin_ws/src/sar_localization/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::sar_localization::Csi_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sar_localization::Csi_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sar_localization::Csi_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sar_localization::Csi_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sar_localization::Csi_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sar_localization::Csi_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::sar_localization::Csi_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d4390fb55572f1efc2a1c6b8190d5e0d";
  }

  static const char* value(const ::sar_localization::Csi_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd4390fb55572f1efULL;
  static const uint64_t static_value2 = 0xc2a1c6b8190d5e0dULL;
};

template<class ContainerAllocator>
struct DataType< ::sar_localization::Csi_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sar_localization/Csi";
  }

  static const char* value(const ::sar_localization::Csi_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sar_localization::Csi_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
uint8 Ntx\n\
std_msgs/Float64MultiArray csi1_real\n\
std_msgs/Float64MultiArray csi1_image\n\
std_msgs/Float64MultiArray csi2_real\n\
std_msgs/Float64MultiArray csi2_image\n\
bool check_csi\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: std_msgs/Float64MultiArray\n\
# Please look at the MultiArrayLayout message definition for\n\
# documentation on all multiarrays.\n\
\n\
MultiArrayLayout  layout        # specification of data layout\n\
float64[]         data          # array of data\n\
\n\
\n\
================================================================================\n\
MSG: std_msgs/MultiArrayLayout\n\
# The multiarray declares a generic multi-dimensional array of a\n\
# particular data type.  Dimensions are ordered from outer most\n\
# to inner most.\n\
\n\
MultiArrayDimension[] dim # Array of dimension properties\n\
uint32 data_offset        # padding bytes at front of data\n\
\n\
# Accessors should ALWAYS be written in terms of dimension stride\n\
# and specified outer-most dimension first.\n\
# \n\
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]\n\
#\n\
# A standard, 3-channel 640x480 image with interleaved color channels\n\
# would be specified as:\n\
#\n\
# dim[0].label  = \"height\"\n\
# dim[0].size   = 480\n\
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)\n\
# dim[1].label  = \"width\"\n\
# dim[1].size   = 640\n\
# dim[1].stride = 3*640 = 1920\n\
# dim[2].label  = \"channel\"\n\
# dim[2].size   = 3\n\
# dim[2].stride = 3\n\
#\n\
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.\n\
================================================================================\n\
MSG: std_msgs/MultiArrayDimension\n\
string label   # label of given dimension\n\
uint32 size    # size of given dimension (in type units)\n\
uint32 stride  # stride of given dimension\n\
";
  }

  static const char* value(const ::sar_localization::Csi_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::sar_localization::Csi_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.Ntx);
      stream.next(m.csi1_real);
      stream.next(m.csi1_image);
      stream.next(m.csi2_real);
      stream.next(m.csi2_image);
      stream.next(m.check_csi);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct Csi_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sar_localization::Csi_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sar_localization::Csi_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "Ntx: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.Ntx);
    s << indent << "csi1_real: ";
    s << std::endl;
    Printer< ::std_msgs::Float64MultiArray_<ContainerAllocator> >::stream(s, indent + "  ", v.csi1_real);
    s << indent << "csi1_image: ";
    s << std::endl;
    Printer< ::std_msgs::Float64MultiArray_<ContainerAllocator> >::stream(s, indent + "  ", v.csi1_image);
    s << indent << "csi2_real: ";
    s << std::endl;
    Printer< ::std_msgs::Float64MultiArray_<ContainerAllocator> >::stream(s, indent + "  ", v.csi2_real);
    s << indent << "csi2_image: ";
    s << std::endl;
    Printer< ::std_msgs::Float64MultiArray_<ContainerAllocator> >::stream(s, indent + "  ", v.csi2_image);
    s << indent << "check_csi: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.check_csi);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SAR_LOCALIZATION_MESSAGE_CSI_H
