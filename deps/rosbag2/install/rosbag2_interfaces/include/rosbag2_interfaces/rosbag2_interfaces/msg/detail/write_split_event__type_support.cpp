// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from rosbag2_interfaces:msg/WriteSplitEvent.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rosbag2_interfaces/msg/detail/write_split_event__struct.hpp"
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

void WriteSplitEvent_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rosbag2_interfaces::msg::WriteSplitEvent(_init);
}

void WriteSplitEvent_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rosbag2_interfaces::msg::WriteSplitEvent *>(message_memory);
  typed_message->~WriteSplitEvent();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember WriteSplitEvent_message_member_array[2] = {
  {
    "closed_file",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosbag2_interfaces::msg::WriteSplitEvent, closed_file),  // bytes offset in struct
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
    offsetof(rosbag2_interfaces::msg::WriteSplitEvent, opened_file),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers WriteSplitEvent_message_members = {
  "rosbag2_interfaces::msg",  // message namespace
  "WriteSplitEvent",  // message name
  2,  // number of fields
  sizeof(rosbag2_interfaces::msg::WriteSplitEvent),
  WriteSplitEvent_message_member_array,  // message members
  WriteSplitEvent_init_function,  // function to initialize message memory (memory has to be allocated)
  WriteSplitEvent_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t WriteSplitEvent_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &WriteSplitEvent_message_members,
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
get_message_type_support_handle<rosbag2_interfaces::msg::WriteSplitEvent>()
{
  return &::rosbag2_interfaces::msg::rosidl_typesupport_introspection_cpp::WriteSplitEvent_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rosbag2_interfaces, msg, WriteSplitEvent)() {
  return &::rosbag2_interfaces::msg::rosidl_typesupport_introspection_cpp::WriteSplitEvent_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
