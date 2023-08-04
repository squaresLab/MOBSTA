// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosbag2_interfaces:srv/TogglePaused.idl
// generated code does not contain a copyright notice

#ifndef ROSBAG2_INTERFACES__SRV__DETAIL__TOGGLE_PAUSED__BUILDER_HPP_
#define ROSBAG2_INTERFACES__SRV__DETAIL__TOGGLE_PAUSED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosbag2_interfaces/srv/detail/toggle_paused__struct.hpp"
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
auto build<::rosbag2_interfaces::srv::TogglePaused_Request>()
{
  return ::rosbag2_interfaces::srv::TogglePaused_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace rosbag2_interfaces


namespace rosbag2_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosbag2_interfaces::srv::TogglePaused_Response>()
{
  return ::rosbag2_interfaces::srv::TogglePaused_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace rosbag2_interfaces

#endif  // ROSBAG2_INTERFACES__SRV__DETAIL__TOGGLE_PAUSED__BUILDER_HPP_
