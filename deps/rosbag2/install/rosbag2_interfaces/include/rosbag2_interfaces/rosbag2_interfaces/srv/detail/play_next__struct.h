// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rosbag2_interfaces:srv/PlayNext.idl
// generated code does not contain a copyright notice

#ifndef ROSBAG2_INTERFACES__SRV__DETAIL__PLAY_NEXT__STRUCT_H_
#define ROSBAG2_INTERFACES__SRV__DETAIL__PLAY_NEXT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/PlayNext in the package rosbag2_interfaces.
typedef struct rosbag2_interfaces__srv__PlayNext_Request
{
  uint8_t structure_needs_at_least_one_member;
} rosbag2_interfaces__srv__PlayNext_Request;

// Struct for a sequence of rosbag2_interfaces__srv__PlayNext_Request.
typedef struct rosbag2_interfaces__srv__PlayNext_Request__Sequence
{
  rosbag2_interfaces__srv__PlayNext_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosbag2_interfaces__srv__PlayNext_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/PlayNext in the package rosbag2_interfaces.
typedef struct rosbag2_interfaces__srv__PlayNext_Response
{
  /// can only play-next while playback is paused
  bool success;
} rosbag2_interfaces__srv__PlayNext_Response;

// Struct for a sequence of rosbag2_interfaces__srv__PlayNext_Response.
typedef struct rosbag2_interfaces__srv__PlayNext_Response__Sequence
{
  rosbag2_interfaces__srv__PlayNext_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosbag2_interfaces__srv__PlayNext_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROSBAG2_INTERFACES__SRV__DETAIL__PLAY_NEXT__STRUCT_H_
