// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosbag2_interfaces:msg/ReadSplitEvent.idl
// generated code does not contain a copyright notice

#ifndef ROSBAG2_INTERFACES__MSG__DETAIL__READ_SPLIT_EVENT__BUILDER_HPP_
#define ROSBAG2_INTERFACES__MSG__DETAIL__READ_SPLIT_EVENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosbag2_interfaces/msg/detail/read_split_event__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosbag2_interfaces
{

namespace msg
{

namespace builder
{

class Init_ReadSplitEvent_opened_file
{
public:
  explicit Init_ReadSplitEvent_opened_file(::rosbag2_interfaces::msg::ReadSplitEvent & msg)
  : msg_(msg)
  {}
  ::rosbag2_interfaces::msg::ReadSplitEvent opened_file(::rosbag2_interfaces::msg::ReadSplitEvent::_opened_file_type arg)
  {
    msg_.opened_file = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosbag2_interfaces::msg::ReadSplitEvent msg_;
};

class Init_ReadSplitEvent_closed_file
{
public:
  Init_ReadSplitEvent_closed_file()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ReadSplitEvent_opened_file closed_file(::rosbag2_interfaces::msg::ReadSplitEvent::_closed_file_type arg)
  {
    msg_.closed_file = std::move(arg);
    return Init_ReadSplitEvent_opened_file(msg_);
  }

private:
  ::rosbag2_interfaces::msg::ReadSplitEvent msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosbag2_interfaces::msg::ReadSplitEvent>()
{
  return rosbag2_interfaces::msg::builder::Init_ReadSplitEvent_closed_file();
}

}  // namespace rosbag2_interfaces

#endif  // ROSBAG2_INTERFACES__MSG__DETAIL__READ_SPLIT_EVENT__BUILDER_HPP_
