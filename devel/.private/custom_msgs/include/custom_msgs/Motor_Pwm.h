// Generated by gencpp from file custom_msgs/Motor_Pwm.msg
// DO NOT EDIT!


#ifndef CUSTOM_MSGS_MESSAGE_MOTOR_PWM_H
#define CUSTOM_MSGS_MESSAGE_MOTOR_PWM_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace custom_msgs
{
template <class ContainerAllocator>
struct Motor_Pwm_
{
  typedef Motor_Pwm_<ContainerAllocator> Type;

  Motor_Pwm_()
    : L(0)
    , R(0)  {
    }
  Motor_Pwm_(const ContainerAllocator& _alloc)
    : L(0)
    , R(0)  {
  (void)_alloc;
    }



   typedef int64_t _L_type;
  _L_type L;

   typedef int64_t _R_type;
  _R_type R;





  typedef boost::shared_ptr< ::custom_msgs::Motor_Pwm_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::custom_msgs::Motor_Pwm_<ContainerAllocator> const> ConstPtr;

}; // struct Motor_Pwm_

typedef ::custom_msgs::Motor_Pwm_<std::allocator<void> > Motor_Pwm;

typedef boost::shared_ptr< ::custom_msgs::Motor_Pwm > Motor_PwmPtr;
typedef boost::shared_ptr< ::custom_msgs::Motor_Pwm const> Motor_PwmConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::custom_msgs::Motor_Pwm_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::custom_msgs::Motor_Pwm_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::custom_msgs::Motor_Pwm_<ContainerAllocator1> & lhs, const ::custom_msgs::Motor_Pwm_<ContainerAllocator2> & rhs)
{
  return lhs.L == rhs.L &&
    lhs.R == rhs.R;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::custom_msgs::Motor_Pwm_<ContainerAllocator1> & lhs, const ::custom_msgs::Motor_Pwm_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace custom_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::custom_msgs::Motor_Pwm_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::custom_msgs::Motor_Pwm_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::custom_msgs::Motor_Pwm_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::custom_msgs::Motor_Pwm_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::custom_msgs::Motor_Pwm_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::custom_msgs::Motor_Pwm_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::custom_msgs::Motor_Pwm_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4184f594ee6fa4706c2c2eca40be03fe";
  }

  static const char* value(const ::custom_msgs::Motor_Pwm_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4184f594ee6fa470ULL;
  static const uint64_t static_value2 = 0x6c2c2eca40be03feULL;
};

template<class ContainerAllocator>
struct DataType< ::custom_msgs::Motor_Pwm_<ContainerAllocator> >
{
  static const char* value()
  {
    return "custom_msgs/Motor_Pwm";
  }

  static const char* value(const ::custom_msgs::Motor_Pwm_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::custom_msgs::Motor_Pwm_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int64 L\n"
"int64 R\n"
;
  }

  static const char* value(const ::custom_msgs::Motor_Pwm_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::custom_msgs::Motor_Pwm_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.L);
      stream.next(m.R);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Motor_Pwm_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::custom_msgs::Motor_Pwm_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::custom_msgs::Motor_Pwm_<ContainerAllocator>& v)
  {
    s << indent << "L: ";
    Printer<int64_t>::stream(s, indent + "  ", v.L);
    s << indent << "R: ";
    Printer<int64_t>::stream(s, indent + "  ", v.R);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CUSTOM_MSGS_MESSAGE_MOTOR_PWM_H
