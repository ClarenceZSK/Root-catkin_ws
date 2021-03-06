// Generated by gencpp from file sar_localization/Motor.msg
// DO NOT EDIT!


#ifndef SAR_LOCALIZATION_MESSAGE_MOTOR_H
#define SAR_LOCALIZATION_MESSAGE_MOTOR_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace sar_localization
{
template <class ContainerAllocator>
struct Motor_
{
  typedef Motor_<ContainerAllocator> Type;

  Motor_()
    : header()
    , std_yaw(0.0)  {
    }
  Motor_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , std_yaw(0.0)  {
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _std_yaw_type;
  _std_yaw_type std_yaw;




  typedef boost::shared_ptr< ::sar_localization::Motor_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::sar_localization::Motor_<ContainerAllocator> const> ConstPtr;

}; // struct Motor_

typedef ::sar_localization::Motor_<std::allocator<void> > Motor;

typedef boost::shared_ptr< ::sar_localization::Motor > MotorPtr;
typedef boost::shared_ptr< ::sar_localization::Motor const> MotorConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sar_localization::Motor_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::sar_localization::Motor_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace sar_localization

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'sar_localization': ['/home/clarence/catkin_ws/src/sar_localization/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::sar_localization::Motor_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sar_localization::Motor_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sar_localization::Motor_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sar_localization::Motor_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sar_localization::Motor_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sar_localization::Motor_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::sar_localization::Motor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "479d24b37cd725e6116c15eda2b65f55";
  }

  static const char* value(const ::sar_localization::Motor_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x479d24b37cd725e6ULL;
  static const uint64_t static_value2 = 0x116c15eda2b65f55ULL;
};

template<class ContainerAllocator>
struct DataType< ::sar_localization::Motor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sar_localization/Motor";
  }

  static const char* value(const ::sar_localization::Motor_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sar_localization::Motor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
float32 std_yaw\n\
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
";
  }

  static const char* value(const ::sar_localization::Motor_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::sar_localization::Motor_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.std_yaw);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct Motor_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sar_localization::Motor_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sar_localization::Motor_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "std_yaw: ";
    Printer<float>::stream(s, indent + "  ", v.std_yaw);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SAR_LOCALIZATION_MESSAGE_MOTOR_H
