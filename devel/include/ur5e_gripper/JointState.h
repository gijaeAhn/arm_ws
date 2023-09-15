// Generated by gencpp from file ur5e_gripper/JointState.msg
// DO NOT EDIT!


#ifndef UR5E_GRIPPER_MESSAGE_JOINTSTATE_H
#define UR5E_GRIPPER_MESSAGE_JOINTSTATE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace ur5e_gripper
{
template <class ContainerAllocator>
struct JointState_
{
  typedef JointState_<ContainerAllocator> Type;

  JointState_()
    : header()
    , name()
    , position()
    , velocity()
    , effort()  {
    }
  JointState_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , name(_alloc)
    , position(_alloc)
    , velocity(_alloc)
    , effort(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> _name_type;
  _name_type name;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _position_type;
  _position_type position;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _velocity_type;
  _velocity_type velocity;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _effort_type;
  _effort_type effort;





  typedef boost::shared_ptr< ::ur5e_gripper::JointState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ur5e_gripper::JointState_<ContainerAllocator> const> ConstPtr;

}; // struct JointState_

typedef ::ur5e_gripper::JointState_<std::allocator<void> > JointState;

typedef boost::shared_ptr< ::ur5e_gripper::JointState > JointStatePtr;
typedef boost::shared_ptr< ::ur5e_gripper::JointState const> JointStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ur5e_gripper::JointState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ur5e_gripper::JointState_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ur5e_gripper::JointState_<ContainerAllocator1> & lhs, const ::ur5e_gripper::JointState_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.name == rhs.name &&
    lhs.position == rhs.position &&
    lhs.velocity == rhs.velocity &&
    lhs.effort == rhs.effort;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ur5e_gripper::JointState_<ContainerAllocator1> & lhs, const ::ur5e_gripper::JointState_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ur5e_gripper

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ur5e_gripper::JointState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur5e_gripper::JointState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ur5e_gripper::JointState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ur5e_gripper::JointState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur5e_gripper::JointState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur5e_gripper::JointState_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ur5e_gripper::JointState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3066dcd76a6cfaef579bd0f34173e9fd";
  }

  static const char* value(const ::ur5e_gripper::JointState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3066dcd76a6cfaefULL;
  static const uint64_t static_value2 = 0x579bd0f34173e9fdULL;
};

template<class ContainerAllocator>
struct DataType< ::ur5e_gripper::JointState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ur5e_gripper/JointState";
  }

  static const char* value(const ::ur5e_gripper::JointState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ur5e_gripper::JointState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"string[] name\n"
"float64[] position\n"
"float64[] velocity\n"
"float64[] effort\n"
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
;
  }

  static const char* value(const ::ur5e_gripper::JointState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ur5e_gripper::JointState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.name);
      stream.next(m.position);
      stream.next(m.velocity);
      stream.next(m.effort);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct JointState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ur5e_gripper::JointState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ur5e_gripper::JointState_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "name[]" << std::endl;
    for (size_t i = 0; i < v.name.size(); ++i)
    {
      s << indent << "  name[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.name[i]);
    }
    s << indent << "position[]" << std::endl;
    for (size_t i = 0; i < v.position.size(); ++i)
    {
      s << indent << "  position[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.position[i]);
    }
    s << indent << "velocity[]" << std::endl;
    for (size_t i = 0; i < v.velocity.size(); ++i)
    {
      s << indent << "  velocity[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.velocity[i]);
    }
    s << indent << "effort[]" << std::endl;
    for (size_t i = 0; i < v.effort.size(); ++i)
    {
      s << indent << "  effort[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.effort[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // UR5E_GRIPPER_MESSAGE_JOINTSTATE_H
