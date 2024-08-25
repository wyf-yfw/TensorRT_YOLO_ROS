// Generated by gencpp from file tensorrt_yolo/Results.msg
// DO NOT EDIT!


#ifndef TENSORRT_YOLO_MESSAGE_RESULTS_H
#define TENSORRT_YOLO_MESSAGE_RESULTS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>
#include <tensorrt_yolo/InferResult.h>

namespace tensorrt_yolo
{
template <class ContainerAllocator>
struct Results_
{
  typedef Results_<ContainerAllocator> Type;

  Results_()
    : results()  {
    }
  Results_(const ContainerAllocator& _alloc)
    : results(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::tensorrt_yolo::InferResult_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::tensorrt_yolo::InferResult_<ContainerAllocator> >> _results_type;
  _results_type results;





  typedef boost::shared_ptr< ::tensorrt_yolo::Results_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tensorrt_yolo::Results_<ContainerAllocator> const> ConstPtr;

}; // struct Results_

typedef ::tensorrt_yolo::Results_<std::allocator<void> > Results;

typedef boost::shared_ptr< ::tensorrt_yolo::Results > ResultsPtr;
typedef boost::shared_ptr< ::tensorrt_yolo::Results const> ResultsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tensorrt_yolo::Results_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tensorrt_yolo::Results_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::tensorrt_yolo::Results_<ContainerAllocator1> & lhs, const ::tensorrt_yolo::Results_<ContainerAllocator2> & rhs)
{
  return lhs.results == rhs.results;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::tensorrt_yolo::Results_<ContainerAllocator1> & lhs, const ::tensorrt_yolo::Results_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace tensorrt_yolo

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::tensorrt_yolo::Results_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tensorrt_yolo::Results_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tensorrt_yolo::Results_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tensorrt_yolo::Results_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tensorrt_yolo::Results_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tensorrt_yolo::Results_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tensorrt_yolo::Results_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b2681add3fd6898c1c0989aee273111b";
  }

  static const char* value(const ::tensorrt_yolo::Results_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb2681add3fd6898cULL;
  static const uint64_t static_value2 = 0x1c0989aee273111bULL;
};

template<class ContainerAllocator>
struct DataType< ::tensorrt_yolo::Results_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tensorrt_yolo/Results";
  }

  static const char* value(const ::tensorrt_yolo::Results_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tensorrt_yolo::Results_<ContainerAllocator> >
{
  static const char* value()
  {
    return "InferResult[] results\n"
"================================================================================\n"
"MSG: tensorrt_yolo/InferResult\n"
"float32[4] bbox\n"
"float32 conf\n"
"int32 classId\n"
"float32[3] coordinate\n"
"int32 Id\n"
"KeyPoint[] kpts\n"
"\n"
"================================================================================\n"
"MSG: tensorrt_yolo/KeyPoint\n"
"float32 x        # x coordinate\n"
"float32 y        # y coordinate\n"
"float32 visible  # visibility (0 or 1)\n"
;
  }

  static const char* value(const ::tensorrt_yolo::Results_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tensorrt_yolo::Results_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.results);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Results_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tensorrt_yolo::Results_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::tensorrt_yolo::Results_<ContainerAllocator>& v)
  {
    s << indent << "results[]" << std::endl;
    for (size_t i = 0; i < v.results.size(); ++i)
    {
      s << indent << "  results[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::tensorrt_yolo::InferResult_<ContainerAllocator> >::stream(s, indent + "    ", v.results[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // TENSORRT_YOLO_MESSAGE_RESULTS_H
