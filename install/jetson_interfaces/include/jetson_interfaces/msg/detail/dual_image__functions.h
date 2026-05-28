// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from jetson_interfaces:msg/DualImage.idl
// generated code does not contain a copyright notice

#ifndef JETSON_INTERFACES__MSG__DETAIL__DUAL_IMAGE__FUNCTIONS_H_
#define JETSON_INTERFACES__MSG__DETAIL__DUAL_IMAGE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "jetson_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "jetson_interfaces/msg/detail/dual_image__struct.h"

/// Initialize msg/DualImage message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * jetson_interfaces__msg__DualImage
 * )) before or use
 * jetson_interfaces__msg__DualImage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_jetson_interfaces
bool
jetson_interfaces__msg__DualImage__init(jetson_interfaces__msg__DualImage * msg);

/// Finalize msg/DualImage message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jetson_interfaces
void
jetson_interfaces__msg__DualImage__fini(jetson_interfaces__msg__DualImage * msg);

/// Create msg/DualImage message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * jetson_interfaces__msg__DualImage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_jetson_interfaces
jetson_interfaces__msg__DualImage *
jetson_interfaces__msg__DualImage__create();

/// Destroy msg/DualImage message.
/**
 * It calls
 * jetson_interfaces__msg__DualImage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jetson_interfaces
void
jetson_interfaces__msg__DualImage__destroy(jetson_interfaces__msg__DualImage * msg);

/// Check for msg/DualImage message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_jetson_interfaces
bool
jetson_interfaces__msg__DualImage__are_equal(const jetson_interfaces__msg__DualImage * lhs, const jetson_interfaces__msg__DualImage * rhs);

/// Copy a msg/DualImage message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_jetson_interfaces
bool
jetson_interfaces__msg__DualImage__copy(
  const jetson_interfaces__msg__DualImage * input,
  jetson_interfaces__msg__DualImage * output);

/// Initialize array of msg/DualImage messages.
/**
 * It allocates the memory for the number of elements and calls
 * jetson_interfaces__msg__DualImage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_jetson_interfaces
bool
jetson_interfaces__msg__DualImage__Sequence__init(jetson_interfaces__msg__DualImage__Sequence * array, size_t size);

/// Finalize array of msg/DualImage messages.
/**
 * It calls
 * jetson_interfaces__msg__DualImage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jetson_interfaces
void
jetson_interfaces__msg__DualImage__Sequence__fini(jetson_interfaces__msg__DualImage__Sequence * array);

/// Create array of msg/DualImage messages.
/**
 * It allocates the memory for the array and calls
 * jetson_interfaces__msg__DualImage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_jetson_interfaces
jetson_interfaces__msg__DualImage__Sequence *
jetson_interfaces__msg__DualImage__Sequence__create(size_t size);

/// Destroy array of msg/DualImage messages.
/**
 * It calls
 * jetson_interfaces__msg__DualImage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jetson_interfaces
void
jetson_interfaces__msg__DualImage__Sequence__destroy(jetson_interfaces__msg__DualImage__Sequence * array);

/// Check for msg/DualImage message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_jetson_interfaces
bool
jetson_interfaces__msg__DualImage__Sequence__are_equal(const jetson_interfaces__msg__DualImage__Sequence * lhs, const jetson_interfaces__msg__DualImage__Sequence * rhs);

/// Copy an array of msg/DualImage messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_jetson_interfaces
bool
jetson_interfaces__msg__DualImage__Sequence__copy(
  const jetson_interfaces__msg__DualImage__Sequence * input,
  jetson_interfaces__msg__DualImage__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // JETSON_INTERFACES__MSG__DETAIL__DUAL_IMAGE__FUNCTIONS_H_
