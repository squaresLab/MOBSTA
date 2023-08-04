// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosbag2_interfaces:srv/IsPaused.idl
// generated code does not contain a copyright notice

#ifndef ROSBAG2_INTERFACES__SRV__DETAIL__IS_PAUSED__BUILDER_HPP_
#define ROSBAG2_INTERFACES__SRV__DETAIL__IS_PAUSED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosbag2_interfaces/srv/detail/is_paused__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosbag2_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosbag2_interfaces::srv::IsPaused_Request>()
{
  return ::rosbag2_interfaces::srv::IsPaused_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace rosbag2_interfaces


namespace rosbag2_interfaces
{

namespace srv
{

namespace builder
{

class Init_IsPaused_Response_paused
{
public:
  Init_IsPaused_Response_paused()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rosbag2_interfaces::srv::IsPaused_Response paused(::rosbag2_interfaces::srv::IsPaused_Response::_paused_type arg)
  {
    msg_.paused = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosbag2_interfaces::srv::IsPaused_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosbag2_interfaces::srv::IsPaused_Response>()
{
  return rosbag2_interfaces::srv::builder::Init_IsPaused_Response_paused();
}

}  // namespace rosbag2_interfaces

#endif  // ROSBAG2_INTERFACES__SRV__DETAIL__IS_PAUSED__BUILDER_HPP_
