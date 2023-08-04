// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rosbag2_interfaces:srv/SetRate.idl
// generated code does not contain a copyright notice

#ifndef ROSBAG2_INTERFACES__SRV__DETAIL__SET_RATE__FUNCTIONS_H_
#define ROSBAG2_INTERFACES__SRV__DETAIL__SET_RATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "rosbag2_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "rosbag2_interfaces/srv/detail/set_rate__struct.h"

/// Initialize srv/SetRate message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rosbag2_interfaces__srv__SetRate_Request
 * )) before or use
 * rosbag2_interfaces__srv__SetRate_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rosbag2_interfaces
bool
rosbag2_interfaces__srv__SetRate_Request__init(rosbag2_interfaces__srv__SetRate_Request * msg);

/// Finalize srv/SetRate message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosbag2_interfaces
void
rosbag2_interfaces__srv__SetRate_Request__fini(rosbag2_interfaces__srv__SetRate_Request * msg);

/// Create srv/SetRate message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rosbag2_interfaces__srv__SetRate_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rosbag2_interfaces
rosbag2_interfaces__srv__SetRate_Request *
rosbag2_interfaces__srv__SetRate_Request__create();

/// Destroy srv/SetRate message.
/**
 * It calls
 * rosbag2_interfaces__srv__SetRate_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosbag2_interfaces
void
rosbag2_interfaces__srv__SetRate_Request__destroy(rosbag2_interfaces__srv__SetRate_Request * msg);

/// Check for srv/SetRate message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosbag2_interfaces
bool
rosbag2_interfaces__srv__SetRate_Request__are_equal(const rosbag2_interfaces__srv__SetRate_Request * lhs, const rosbag2_interfaces__srv__SetRate_Request * rhs);

/// Copy a srv/SetRate message.
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
ROSIDL_GENERATOR_C_PUBLIC_rosbag2_interfaces
bool
rosbag2_interfaces__srv__SetRate_Request__copy(
  const rosbag2_interfaces__srv__SetRate_Request * input,
  rosbag2_interfaces__srv__SetRate_Request * output);

/// Initialize array of srv/SetRate messages.
/**
 * It allocates the memory for the number of elements and calls
 * rosbag2_interfaces__srv__SetRate_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosbag2_interfaces
bool
rosbag2_interfaces__srv__SetRate_Request__Sequence__init(rosbag2_interfaces__srv__SetRate_Request__Sequence * array, size_t size);

/// Finalize array of srv/SetRate messages.
/**
 * It calls
 * rosbag2_interfaces__srv__SetRate_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosbag2_interfaces
void
rosbag2_interfaces__srv__SetRate_Request__Sequence__fini(rosbag2_interfaces__srv__SetRate_Request__Sequence * array);

/// Create array of srv/SetRate messages.
/**
 * It allocates the memory for the array and calls
 * rosbag2_interfaces__srv__SetRate_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rosbag2_interfaces
rosbag2_interfaces__srv__SetRate_Request__Sequence *
rosbag2_interfaces__srv__SetRate_Request__Sequence__create(size_t size);

/// Destroy array of srv/SetRate messages.
/**
 * It calls
 * rosbag2_interfaces__srv__SetRate_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosbag2_interfaces
void
rosbag2_interfaces__srv__SetRate_Request__Sequence__destroy(rosbag2_interfaces__srv__SetRate_Request__Sequence * array);

/// Check for srv/SetRate message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosbag2_interfaces
bool
rosbag2_interfaces__srv__SetRate_Request__Sequence__are_equal(const rosbag2_interfaces__srv__SetRate_Request__Sequence * lhs, const rosbag2_interfaces__srv__SetRate_Request__Sequence * rhs);

/// Copy an array of srv/SetRate messages.
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
ROSIDL_GENERATOR_C_PUBLIC_rosbag2_interfaces
bool
rosbag2_interfaces__srv__SetRate_Request__Sequence__copy(
  const rosbag2_interfaces__srv__SetRate_Request__Sequence * input,
  rosbag2_interfaces__srv__SetRate_Request__Sequence * output);

/// Initialize srv/SetRate message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rosbag2_interfaces__srv__SetRate_Response
 * )) before or use
 * rosbag2_interfaces__srv__SetRate_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rosbag2_interfaces
bool
rosbag2_interfaces__srv__SetRate_Response__init(rosbag2_interfaces__srv__SetRate_Response * msg);

/// Finalize srv/SetRate message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosbag2_interfaces
void
rosbag2_interfaces__srv__SetRate_Response__fini(rosbag2_interfaces__srv__SetRate_Response * msg);

/// Create srv/SetRate message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rosbag2_interfaces__srv__SetRate_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rosbag2_interfaces
rosbag2_interfaces__srv__SetRate_Response *
rosbag2_interfaces__srv__SetRate_Response__create();

/// Destroy srv/SetRate message.
/**
 * It calls
 * rosbag2_interfaces__srv__SetRate_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosbag2_interfaces
void
rosbag2_interfaces__srv__SetRate_Response__destroy(rosbag2_interfaces__srv__SetRate_Response * msg);

/// Check for srv/SetRate message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosbag2_interfaces
bool
rosbag2_interfaces__srv__SetRate_Response__are_equal(const rosbag2_interfaces__srv__SetRate_Response * lhs, const rosbag2_interfaces__srv__SetRate_Response * rhs);

/// Copy a srv/SetRate message.
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
ROSIDL_GENERATOR_C_PUBLIC_rosbag2_interfaces
bool
rosbag2_interfaces__srv__SetRate_Response__copy(
  const rosbag2_interfaces__srv__SetRate_Response * input,
  rosbag2_interfaces__srv__SetRate_Response * output);

/// Initialize array of srv/SetRate messages.
/**
 * It allocates the memory for the number of elements and calls
 * rosbag2_interfaces__srv__SetRate_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosbag2_interfaces
bool
rosbag2_interfaces__srv__SetRate_Response__Sequence__init(rosbag2_interfaces__srv__SetRate_Response__Sequence * array, size_t size);

/// Finalize array of srv/SetRate messages.
/**
 * It calls
 * rosbag2_interfaces__srv__SetRate_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosbag2_interfaces
void
rosbag2_interfaces__srv__SetRate_Response__Sequence__fini(rosbag2_interfaces__srv__SetRate_Response__Sequence * array);

/// Create array of srv/SetRate messages.
/**
 * It allocates the memory for the array and calls
 * rosbag2_interfaces__srv__SetRate_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rosbag2_interfaces
rosbag2_interfaces__srv__SetRate_Response__Sequence *
rosbag2_interfaces__srv__SetRate_Response__Sequence__create(size_t size);

/// Destroy array of srv/SetRate messages.
/**
 * It calls
 * rosbag2_interfaces__srv__SetRate_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosbag2_interfaces
void
rosbag2_interfaces__srv__SetRate_Response__Sequence__destroy(rosbag2_interfaces__srv__SetRate_Response__Sequence * array);

/// Check for srv/SetRate message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosbag2_interfaces
bool
rosbag2_interfaces__srv__SetRate_Response__Sequence__are_equal(const rosbag2_interfaces__srv__SetRate_Response__Sequence * lhs, const rosbag2_interfaces__srv__SetRate_Response__Sequence * rhs);

/// Copy an array of srv/SetRate messages.
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
ROSIDL_GENERATOR_C_PUBLIC_rosbag2_interfaces
bool
rosbag2_interfaces__srv__SetRate_Response__Sequence__copy(
  const rosbag2_interfaces__srv__SetRate_Response__Sequence * input,
  rosbag2_interfaces__srv__SetRate_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROSBAG2_INTERFACES__SRV__DETAIL__SET_RATE__FUNCTIONS_H_
