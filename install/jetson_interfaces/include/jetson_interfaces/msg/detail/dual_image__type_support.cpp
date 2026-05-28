// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from jetson_interfaces:msg/DualImage.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "jetson_interfaces/msg/detail/dual_image__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace jetson_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void DualImage_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) jetson_interfaces::msg::DualImage(_init);
}

void DualImage_fini_function(void * message_memory)
{
  auto typed_message = static_cast<jetson_interfaces::msg::DualImage *>(message_memory);
  typed_message->~DualImage();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember DualImage_message_member_array[2] = {
  {
    "raw_image",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sensor_msgs::msg::CompressedImage>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jetson_interfaces::msg::DualImage, raw_image),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "undistorted_image",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sensor_msgs::msg::CompressedImage>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jetson_interfaces::msg::DualImage, undistorted_image),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers DualImage_message_members = {
  "jetson_interfaces::msg",  // message namespace
  "DualImage",  // message name
  2,  // number of fields
  sizeof(jetson_interfaces::msg::DualImage),
  DualImage_message_member_array,  // message members
  DualImage_init_function,  // function to initialize message memory (memory has to be allocated)
  DualImage_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t DualImage_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &DualImage_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace jetson_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<jetson_interfaces::msg::DualImage>()
{
  return &::jetson_interfaces::msg::rosidl_typesupport_introspection_cpp::DualImage_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, jetson_interfaces, msg, DualImage)() {
  return &::jetson_interfaces::msg::rosidl_typesupport_introspection_cpp::DualImage_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
