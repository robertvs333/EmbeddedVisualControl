// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from jetson_interfaces:msg/DualImage.idl
// generated code does not contain a copyright notice

#ifndef JETSON_INTERFACES__MSG__DETAIL__DUAL_IMAGE__TRAITS_HPP_
#define JETSON_INTERFACES__MSG__DETAIL__DUAL_IMAGE__TRAITS_HPP_

#include "jetson_interfaces/msg/detail/dual_image__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'raw_image'
// Member 'undistorted_image'
#include "sensor_msgs/msg/detail/compressed_image__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<jetson_interfaces::msg::DualImage>()
{
  return "jetson_interfaces::msg::DualImage";
}

template<>
inline const char * name<jetson_interfaces::msg::DualImage>()
{
  return "jetson_interfaces/msg/DualImage";
}

template<>
struct has_fixed_size<jetson_interfaces::msg::DualImage>
  : std::integral_constant<bool, has_fixed_size<sensor_msgs::msg::CompressedImage>::value> {};

template<>
struct has_bounded_size<jetson_interfaces::msg::DualImage>
  : std::integral_constant<bool, has_bounded_size<sensor_msgs::msg::CompressedImage>::value> {};

template<>
struct is_message<jetson_interfaces::msg::DualImage>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // JETSON_INTERFACES__MSG__DETAIL__DUAL_IMAGE__TRAITS_HPP_
