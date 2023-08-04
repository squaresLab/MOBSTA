// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rosbag2_interfaces:msg/WriteSplitEvent.idl
// generated code does not contain a copyright notice

#ifndef ROSBAG2_INTERFACES__MSG__DETAIL__WRITE_SPLIT_EVENT__STRUCT_H_
#define ROSBAG2_INTERFACES__MSG__DETAIL__WRITE_SPLIT_EVENT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'closed_file'
// Member 'opened_file'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/WriteSplitEvent in the package rosbag2_interfaces.
/**
  * The full path of the file that was finished and closed
 */
typedef struct rosbag2_interfaces__msg__WriteSplitEvent
{
  rosidl_runtime_c__String closed_file;
  /// The full path of the new file that was created to continue recording
  rosidl_runtime_c__String opened_file;
} rosbag2_interfaces__msg__WriteSplitEvent;

// Struct for a sequence of rosbag2_interfaces__msg__WriteSplitEvent.
typedef struct rosbag2_interfaces__msg__WriteSplitEvent__Sequence
{
  rosbag2_interfaces__msg__WriteSplitEvent * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosbag2_interfaces__msg__WriteSplitEvent__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROSBAG2_INTERFACES__MSG__DETAIL__WRITE_SPLIT_EVENT__STRUCT_H_
