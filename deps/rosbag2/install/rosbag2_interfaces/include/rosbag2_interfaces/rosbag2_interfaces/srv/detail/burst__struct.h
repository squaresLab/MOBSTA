// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rosbag2_interfaces:srv/Burst.idl
// generated code does not contain a copyright notice

#ifndef ROSBAG2_INTERFACES__SRV__DETAIL__BURST__STRUCT_H_
#define ROSBAG2_INTERFACES__SRV__DETAIL__BURST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Burst in the package rosbag2_interfaces.
typedef struct rosbag2_interfaces__srv__Burst_Request
{
  /// Number of messages to burst
  uint64_t num_messages;
} rosbag2_interfaces__srv__Burst_Request;

// Struct for a sequence of rosbag2_interfaces__srv__Burst_Request.
typedef struct rosbag2_interfaces__srv__Burst_Request__Sequence
{
  rosbag2_interfaces__srv__Burst_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosbag2_interfaces__srv__Burst_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/Burst in the package rosbag2_interfaces.
typedef struct rosbag2_interfaces__srv__Burst_Response
{
  /// Number of messages actually burst
  uint64_t actually_burst;
} rosbag2_interfaces__srv__Burst_Response;

// Struct for a sequence of rosbag2_interfaces__srv__Burst_Response.
typedef struct rosbag2_interfaces__srv__Burst_Response__Sequence
{
  rosbag2_interfaces__srv__Burst_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosbag2_interfaces__srv__Burst_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROSBAG2_INTERFACES__SRV__DETAIL__BURST__STRUCT_H_
