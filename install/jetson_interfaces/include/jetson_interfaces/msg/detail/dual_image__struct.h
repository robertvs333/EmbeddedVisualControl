// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jetson_interfaces:msg/DualImage.idl
// generated code does not contain a copyright notice

#ifndef JETSON_INTERFACES__MSG__DETAIL__DUAL_IMAGE__STRUCT_H_
#define JETSON_INTERFACES__MSG__DETAIL__DUAL_IMAGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'raw_image'
// Member 'undistorted_image'
#include "sensor_msgs/msg/detail/compressed_image__struct.h"

// Struct defined in msg/DualImage in the package jetson_interfaces.
typedef struct jetson_interfaces__msg__DualImage
{
  sensor_msgs__msg__CompressedImage raw_image;
  sensor_msgs__msg__CompressedImage undistorted_image;
} jetson_interfaces__msg__DualImage;

// Struct for a sequence of jetson_interfaces__msg__DualImage.
typedef struct jetson_interfaces__msg__DualImage__Sequence
{
  jetson_interfaces__msg__DualImage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jetson_interfaces__msg__DualImage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JETSON_INTERFACES__MSG__DETAIL__DUAL_IMAGE__STRUCT_H_
