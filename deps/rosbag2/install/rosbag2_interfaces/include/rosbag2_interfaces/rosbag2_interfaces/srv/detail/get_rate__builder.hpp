// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosbag2_interfaces:srv/GetRate.idl
// generated code does not contain a copyright notice

#ifndef ROSBAG2_INTERFACES__SRV__DETAIL__GET_RATE__BUILDER_HPP_
#define ROSBAG2_INTERFACES__SRV__DETAIL__GET_RATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosbag2_interfaces/srv/detail/get_rate__struct.hpp"
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
auto build<::rosbag2_interfaces::srv::GetRate_Request>()
{
  return ::rosbag2_interfaces::srv::GetRate_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace rosbag2_interfaces


namespace rosbag2_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetRate_Response_rate
{
public:
  Init_GetRate_Response_rate()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rosbag2_interfaces::srv::GetRate_Response rate(::rosbag2_interfaces::srv::GetRate_Response::_rate_type arg)
  {
    msg_.rate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosbag2_interfaces::srv::GetRate_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosbag2_interfaces::srv::GetRate_Response>()
{
  return rosbag2_interfaces::srv::builder::Init_GetRate_Response_rate();
}

}  // namespace rosbag2_interfaces

#endif  // ROSBAG2_INTERFACES__SRV__DETAIL__GET_RATE__BUILDER_HPP_
