// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosbag2_interfaces:srv/Snapshot.idl
// generated code does not contain a copyright notice

#ifndef ROSBAG2_INTERFACES__SRV__DETAIL__SNAPSHOT__BUILDER_HPP_
#define ROSBAG2_INTERFACES__SRV__DETAIL__SNAPSHOT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosbag2_interfaces/srv/detail/snapshot__struct.hpp"
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
auto build<::rosbag2_interfaces::srv::Snapshot_Request>()
{
  return ::rosbag2_interfaces::srv::Snapshot_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace rosbag2_interfaces


namespace rosbag2_interfaces
{

namespace srv
{

namespace builder
{

class Init_Snapshot_Response_success
{
public:
  Init_Snapshot_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rosbag2_interfaces::srv::Snapshot_Response success(::rosbag2_interfaces::srv::Snapshot_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosbag2_interfaces::srv::Snapshot_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosbag2_interfaces::srv::Snapshot_Response>()
{
  return rosbag2_interfaces::srv::builder::Init_Snapshot_Response_success();
}

}  // namespace rosbag2_interfaces

#endif  // ROSBAG2_INTERFACES__SRV__DETAIL__SNAPSHOT__BUILDER_HPP_
