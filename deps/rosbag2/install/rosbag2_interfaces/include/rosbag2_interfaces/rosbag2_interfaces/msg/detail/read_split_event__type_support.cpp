// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from rosbag2_interfaces:msg/ReadSplitEvent.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rosbag2_interfaces/msg/detail/read_split_event__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace rosbag2_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ReadSplitEvent_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rosbag2_interfaces::msg::ReadSplitEvent(_init);
}

void ReadSplitEvent_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rosbag2_interfaces::msg::ReadSplitEvent *>(message_memory);
  typed_message->~ReadSplitEvent();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ReadSplitEvent_message_member_array[2] = {
  {
    "closed_file",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosbag2_interfaces::msg::ReadSplitEvent, closed_file),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "opened_file",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosbag2_interfaces::msg::ReadSplitEvent, opened_file),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ReadSplitEvent_message_members = {
  "rosbag2_interfaces::msg",  // message namespace
  "ReadSplitEvent",  // message name
  2,  // number of fields
  sizeof(rosbag2_interfaces::msg::ReadSplitEvent),
  ReadSplitEvent_message_member_array,  // message members
  ReadSplitEvent_init_function,  // function to initialize message memory (memory has to be allocated)
  ReadSplitEvent_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ReadSplitEvent_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ReadSplitEvent_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace rosbag2_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rosbag2_interfaces::msg::ReadSplitEvent>()
{
  return &::rosbag2_interfaces::msg::rosidl_typesupport_introspection_cpp::ReadSplitEvent_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rosbag2_interfaces, msg, ReadSplitEvent)() {
  return &::rosbag2_interfaces::msg::rosidl_typesupport_introspection_cpp::ReadSplitEvent_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
