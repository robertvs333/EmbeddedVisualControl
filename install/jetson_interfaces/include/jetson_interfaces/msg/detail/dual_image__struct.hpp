// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from jetson_interfaces:msg/DualImage.idl
// generated code does not contain a copyright notice

#ifndef JETSON_INTERFACES__MSG__DETAIL__DUAL_IMAGE__STRUCT_HPP_
#define JETSON_INTERFACES__MSG__DETAIL__DUAL_IMAGE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'raw_image'
// Member 'undistorted_image'
#include "sensor_msgs/msg/detail/compressed_image__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__jetson_interfaces__msg__DualImage __attribute__((deprecated))
#else
# define DEPRECATED__jetson_interfaces__msg__DualImage __declspec(deprecated)
#endif

namespace jetson_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DualImage_
{
  using Type = DualImage_<ContainerAllocator>;

  explicit DualImage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : raw_image(_init),
    undistorted_image(_init)
  {
    (void)_init;
  }

  explicit DualImage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : raw_image(_alloc, _init),
    undistorted_image(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _raw_image_type =
    sensor_msgs::msg::CompressedImage_<ContainerAllocator>;
  _raw_image_type raw_image;
  using _undistorted_image_type =
    sensor_msgs::msg::CompressedImage_<ContainerAllocator>;
  _undistorted_image_type undistorted_image;

  // setters for named parameter idiom
  Type & set__raw_image(
    const sensor_msgs::msg::CompressedImage_<ContainerAllocator> & _arg)
  {
    this->raw_image = _arg;
    return *this;
  }
  Type & set__undistorted_image(
    const sensor_msgs::msg::CompressedImage_<ContainerAllocator> & _arg)
  {
    this->undistorted_image = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    jetson_interfaces::msg::DualImage_<ContainerAllocator> *;
  using ConstRawPtr =
    const jetson_interfaces::msg::DualImage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jetson_interfaces::msg::DualImage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jetson_interfaces::msg::DualImage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jetson_interfaces::msg::DualImage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jetson_interfaces::msg::DualImage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jetson_interfaces::msg::DualImage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jetson_interfaces::msg::DualImage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jetson_interfaces::msg::DualImage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jetson_interfaces::msg::DualImage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jetson_interfaces__msg__DualImage
    std::shared_ptr<jetson_interfaces::msg::DualImage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jetson_interfaces__msg__DualImage
    std::shared_ptr<jetson_interfaces::msg::DualImage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DualImage_ & other) const
  {
    if (this->raw_image != other.raw_image) {
      return false;
    }
    if (this->undistorted_image != other.undistorted_image) {
      return false;
    }
    return true;
  }
  bool operator!=(const DualImage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DualImage_

// alias to use template instance with default allocator
using DualImage =
  jetson_interfaces::msg::DualImage_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace jetson_interfaces

#endif  // JETSON_INTERFACES__MSG__DETAIL__DUAL_IMAGE__STRUCT_HPP_
