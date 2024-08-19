// Generated by gencpp from file tensorrt_yolo/infer_result.msg
// DO NOT EDIT!


#ifndef TENSORRT_YOLO_MESSAGE_INFER_RESULT_H
#define TENSORRT_YOLO_MESSAGE_INFER_RESULT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace tensorrt_yolo
{
template <class ContainerAllocator>
struct infer_result_
{
  typedef infer_result_<ContainerAllocator> Type;

  infer_result_()
    : bbox()
    , conf(0.0)
    , classId(0)
    , coordinate()
    , Id(0)  {
      bbox.assign(0.0);

      coordinate.assign(0.0);
  }
  infer_result_(const ContainerAllocator& _alloc)
    : bbox()
    , conf(0.0)
    , classId(0)
    , coordinate()
    , Id(0)  {
  (void)_alloc;
      bbox.assign(0.0);

      coordinate.assign(0.0);
  }



   typedef boost::array<float, 4>  _bbox_type;
  _bbox_type bbox;

   typedef float _conf_type;
  _conf_type conf;

   typedef int32_t _classId_type;
  _classId_type classId;

   typedef boost::array<float, 3>  _coordinate_type;
  _coordinate_type coordinate;

   typedef int32_t _Id_type;
  _Id_type Id;





  typedef boost::shared_ptr< ::tensorrt_yolo::infer_result_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tensorrt_yolo::infer_result_<ContainerAllocator> const> ConstPtr;

}; // struct infer_result_

typedef ::tensorrt_yolo::infer_result_<std::allocator<void> > infer_result;

typedef boost::shared_ptr< ::tensorrt_yolo::infer_result > infer_resultPtr;
typedef boost::shared_ptr< ::tensorrt_yolo::infer_result const> infer_resultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tensorrt_yolo::infer_result_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tensorrt_yolo::infer_result_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::tensorrt_yolo::infer_result_<ContainerAllocator1> & lhs, const ::tensorrt_yolo::infer_result_<ContainerAllocator2> & rhs)
{
  return lhs.bbox == rhs.bbox &&
    lhs.conf == rhs.conf &&
    lhs.classId == rhs.classId &&
    lhs.coordinate == rhs.coordinate &&
    lhs.Id == rhs.Id;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::tensorrt_yolo::infer_result_<ContainerAllocator1> & lhs, const ::tensorrt_yolo::infer_result_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace tensorrt_yolo

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::tensorrt_yolo::infer_result_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tensorrt_yolo::infer_result_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tensorrt_yolo::infer_result_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tensorrt_yolo::infer_result_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tensorrt_yolo::infer_result_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tensorrt_yolo::infer_result_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tensorrt_yolo::infer_result_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a85073f6fdd0e1f565fa60ab34fa725c";
  }

  static const char* value(const ::tensorrt_yolo::infer_result_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa85073f6fdd0e1f5ULL;
  static const uint64_t static_value2 = 0x65fa60ab34fa725cULL;
};

template<class ContainerAllocator>
struct DataType< ::tensorrt_yolo::infer_result_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tensorrt_yolo/infer_result";
  }

  static const char* value(const ::tensorrt_yolo::infer_result_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tensorrt_yolo::infer_result_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32[4] bbox\n"
"float32 conf\n"
"int32 classId\n"
"float32[3] coordinate\n"
"int32 Id\n"
;
  }

  static const char* value(const ::tensorrt_yolo::infer_result_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tensorrt_yolo::infer_result_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.bbox);
      stream.next(m.conf);
      stream.next(m.classId);
      stream.next(m.coordinate);
      stream.next(m.Id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct infer_result_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tensorrt_yolo::infer_result_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::tensorrt_yolo::infer_result_<ContainerAllocator>& v)
  {
    s << indent << "bbox[]" << std::endl;
    for (size_t i = 0; i < v.bbox.size(); ++i)
    {
      s << indent << "  bbox[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.bbox[i]);
    }
    s << indent << "conf: ";
    Printer<float>::stream(s, indent + "  ", v.conf);
    s << indent << "classId: ";
    Printer<int32_t>::stream(s, indent + "  ", v.classId);
    s << indent << "coordinate[]" << std::endl;
    for (size_t i = 0; i < v.coordinate.size(); ++i)
    {
      s << indent << "  coordinate[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.coordinate[i]);
    }
    s << indent << "Id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.Id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TENSORRT_YOLO_MESSAGE_INFER_RESULT_H
