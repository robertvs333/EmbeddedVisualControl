// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from jetson_interfaces:msg/DualImage.idl
// generated code does not contain a copyright notice
#include "jetson_interfaces/msg/detail/dual_image__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `raw_image`
// Member `undistorted_image`
#include "sensor_msgs/msg/detail/compressed_image__functions.h"

bool
jetson_interfaces__msg__DualImage__init(jetson_interfaces__msg__DualImage * msg)
{
  if (!msg) {
    return false;
  }
  // raw_image
  if (!sensor_msgs__msg__CompressedImage__init(&msg->raw_image)) {
    jetson_interfaces__msg__DualImage__fini(msg);
    return false;
  }
  // undistorted_image
  if (!sensor_msgs__msg__CompressedImage__init(&msg->undistorted_image)) {
    jetson_interfaces__msg__DualImage__fini(msg);
    return false;
  }
  return true;
}

void
jetson_interfaces__msg__DualImage__fini(jetson_interfaces__msg__DualImage * msg)
{
  if (!msg) {
    return;
  }
  // raw_image
  sensor_msgs__msg__CompressedImage__fini(&msg->raw_image);
  // undistorted_image
  sensor_msgs__msg__CompressedImage__fini(&msg->undistorted_image);
}

bool
jetson_interfaces__msg__DualImage__are_equal(const jetson_interfaces__msg__DualImage * lhs, const jetson_interfaces__msg__DualImage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // raw_image
  if (!sensor_msgs__msg__CompressedImage__are_equal(
      &(lhs->raw_image), &(rhs->raw_image)))
  {
    return false;
  }
  // undistorted_image
  if (!sensor_msgs__msg__CompressedImage__are_equal(
      &(lhs->undistorted_image), &(rhs->undistorted_image)))
  {
    return false;
  }
  return true;
}

bool
jetson_interfaces__msg__DualImage__copy(
  const jetson_interfaces__msg__DualImage * input,
  jetson_interfaces__msg__DualImage * output)
{
  if (!input || !output) {
    return false;
  }
  // raw_image
  if (!sensor_msgs__msg__CompressedImage__copy(
      &(input->raw_image), &(output->raw_image)))
  {
    return false;
  }
  // undistorted_image
  if (!sensor_msgs__msg__CompressedImage__copy(
      &(input->undistorted_image), &(output->undistorted_image)))
  {
    return false;
  }
  return true;
}

jetson_interfaces__msg__DualImage *
jetson_interfaces__msg__DualImage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jetson_interfaces__msg__DualImage * msg = (jetson_interfaces__msg__DualImage *)allocator.allocate(sizeof(jetson_interfaces__msg__DualImage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jetson_interfaces__msg__DualImage));
  bool success = jetson_interfaces__msg__DualImage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jetson_interfaces__msg__DualImage__destroy(jetson_interfaces__msg__DualImage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jetson_interfaces__msg__DualImage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jetson_interfaces__msg__DualImage__Sequence__init(jetson_interfaces__msg__DualImage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jetson_interfaces__msg__DualImage * data = NULL;

  if (size) {
    data = (jetson_interfaces__msg__DualImage *)allocator.zero_allocate(size, sizeof(jetson_interfaces__msg__DualImage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jetson_interfaces__msg__DualImage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jetson_interfaces__msg__DualImage__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
jetson_interfaces__msg__DualImage__Sequence__fini(jetson_interfaces__msg__DualImage__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      jetson_interfaces__msg__DualImage__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

jetson_interfaces__msg__DualImage__Sequence *
jetson_interfaces__msg__DualImage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jetson_interfaces__msg__DualImage__Sequence * array = (jetson_interfaces__msg__DualImage__Sequence *)allocator.allocate(sizeof(jetson_interfaces__msg__DualImage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jetson_interfaces__msg__DualImage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jetson_interfaces__msg__DualImage__Sequence__destroy(jetson_interfaces__msg__DualImage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jetson_interfaces__msg__DualImage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jetson_interfaces__msg__DualImage__Sequence__are_equal(const jetson_interfaces__msg__DualImage__Sequence * lhs, const jetson_interfaces__msg__DualImage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jetson_interfaces__msg__DualImage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jetson_interfaces__msg__DualImage__Sequence__copy(
  const jetson_interfaces__msg__DualImage__Sequence * input,
  jetson_interfaces__msg__DualImage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jetson_interfaces__msg__DualImage);
    jetson_interfaces__msg__DualImage * data =
      (jetson_interfaces__msg__DualImage *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jetson_interfaces__msg__DualImage__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          jetson_interfaces__msg__DualImage__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jetson_interfaces__msg__DualImage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
