// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jetson_interfaces:msg/DualImage.idl
// generated code does not contain a copyright notice

#ifndef JETSON_INTERFACES__MSG__DETAIL__DUAL_IMAGE__BUILDER_HPP_
#define JETSON_INTERFACES__MSG__DETAIL__DUAL_IMAGE__BUILDER_HPP_

#include "jetson_interfaces/msg/detail/dual_image__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace jetson_interfaces
{

namespace msg
{

namespace builder
{

class Init_DualImage_undistorted_image
{
public:
  explicit Init_DualImage_undistorted_image(::jetson_interfaces::msg::DualImage & msg)
  : msg_(msg)
  {}
  ::jetson_interfaces::msg::DualImage undistorted_image(::jetson_interfaces::msg::DualImage::_undistorted_image_type arg)
  {
    msg_.undistorted_image = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jetson_interfaces::msg::DualImage msg_;
};

class Init_DualImage_raw_image
{
public:
  Init_DualImage_raw_image()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DualImage_undistorted_image raw_image(::jetson_interfaces::msg::DualImage::_raw_image_type arg)
  {
    msg_.raw_image = std::move(arg);
    return Init_DualImage_undistorted_image(msg_);
  }

private:
  ::jetson_interfaces::msg::DualImage msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::jetson_interfaces::msg::DualImage>()
{
  return jetson_interfaces::msg::builder::Init_DualImage_raw_image();
}

}  // namespace jetson_interfaces

#endif  // JETSON_INTERFACES__MSG__DETAIL__DUAL_IMAGE__BUILDER_HPP_
