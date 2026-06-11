// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from jetson_interfaces:msg/DualImage.idl
// generated code does not contain a copyright notice

#ifndef JETSON_INTERFACES__MSG__DETAIL__DUAL_IMAGE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define JETSON_INTERFACES__MSG__DETAIL__DUAL_IMAGE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "jetson_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "jetson_interfaces/msg/detail/dual_image__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace jetson_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_jetson_interfaces
cdr_serialize(
  const jetson_interfaces::msg::DualImage & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_jetson_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  jetson_interfaces::msg::DualImage & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_jetson_interfaces
get_serialized_size(
  const jetson_interfaces::msg::DualImage & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_jetson_interfaces
max_serialized_size_DualImage(
  bool & full_bounded,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace jetson_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_jetson_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, jetson_interfaces, msg, DualImage)();

#ifdef __cplusplus
}
#endif

#endif  // JETSON_INTERFACES__MSG__DETAIL__DUAL_IMAGE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
