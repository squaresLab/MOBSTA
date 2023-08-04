// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosbag2_interfaces:srv/Burst.idl
// generated code does not contain a copyright notice

#ifndef ROSBAG2_INTERFACES__SRV__DETAIL__BURST__BUILDER_HPP_
#define ROSBAG2_INTERFACES__SRV__DETAIL__BURST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosbag2_interfaces/srv/detail/burst__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosbag2_interfaces
{

namespace srv
{

namespace builder
{

class Init_Burst_Request_num_messages
{
public:
  Init_Burst_Request_num_messages()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rosbag2_interfaces::srv::Burst_Request num_messages(::rosbag2_interfaces::srv::Burst_Request::_num_messages_type arg)
  {
    msg_.num_messages = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosbag2_interfaces::srv::Burst_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosbag2_interfaces::srv::Burst_Request>()
{
  return rosbag2_interfaces::srv::builder::Init_Burst_Request_num_messages();
}

}  // namespace rosbag2_interfaces


namespace rosbag2_interfaces
{

namespace srv
{

namespace builder
{

class Init_Burst_Response_actually_burst
{
public:
  Init_Burst_Response_actually_burst()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rosbag2_interfaces::srv::Burst_Response actually_burst(::rosbag2_interfaces::srv::Burst_Response::_actually_burst_type arg)
  {
    msg_.actually_burst = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosbag2_interfaces::srv::Burst_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosbag2_interfaces::srv::Burst_Response>()
{
  return rosbag2_interfaces::srv::builder::Init_Burst_Response_actually_burst();
}

}  // namespace rosbag2_interfaces

#endif  // ROSBAG2_INTERFACES__SRV__DETAIL__BURST__BUILDER_HPP_
