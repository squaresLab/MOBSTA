// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rosbag2_interfaces:srv/TogglePaused.idl
// generated code does not contain a copyright notice

#ifndef ROSBAG2_INTERFACES__SRV__DETAIL__TOGGLE_PAUSED__STRUCT_HPP_
#define ROSBAG2_INTERFACES__SRV__DETAIL__TOGGLE_PAUSED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rosbag2_interfaces__srv__TogglePaused_Request __attribute__((deprecated))
#else
# define DEPRECATED__rosbag2_interfaces__srv__TogglePaused_Request __declspec(deprecated)
#endif

namespace rosbag2_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TogglePaused_Request_
{
  using Type = TogglePaused_Request_<ContainerAllocator>;

  explicit TogglePaused_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit TogglePaused_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    rosbag2_interfaces::srv::TogglePaused_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosbag2_interfaces::srv::TogglePaused_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosbag2_interfaces::srv::TogglePaused_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosbag2_interfaces::srv::TogglePaused_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosbag2_interfaces::srv::TogglePaused_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosbag2_interfaces::srv::TogglePaused_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosbag2_interfaces::srv::TogglePaused_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosbag2_interfaces::srv::TogglePaused_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosbag2_interfaces::srv::TogglePaused_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosbag2_interfaces::srv::TogglePaused_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosbag2_interfaces__srv__TogglePaused_Request
    std::shared_ptr<rosbag2_interfaces::srv::TogglePaused_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosbag2_interfaces__srv__TogglePaused_Request
    std::shared_ptr<rosbag2_interfaces::srv::TogglePaused_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TogglePaused_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const TogglePaused_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TogglePaused_Request_

// alias to use template instance with default allocator
using TogglePaused_Request =
  rosbag2_interfaces::srv::TogglePaused_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rosbag2_interfaces


#ifndef _WIN32
# define DEPRECATED__rosbag2_interfaces__srv__TogglePaused_Response __attribute__((deprecated))
#else
# define DEPRECATED__rosbag2_interfaces__srv__TogglePaused_Response __declspec(deprecated)
#endif

namespace rosbag2_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TogglePaused_Response_
{
  using Type = TogglePaused_Response_<ContainerAllocator>;

  explicit TogglePaused_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit TogglePaused_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    rosbag2_interfaces::srv::TogglePaused_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosbag2_interfaces::srv::TogglePaused_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosbag2_interfaces::srv::TogglePaused_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosbag2_interfaces::srv::TogglePaused_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosbag2_interfaces::srv::TogglePaused_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosbag2_interfaces::srv::TogglePaused_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosbag2_interfaces::srv::TogglePaused_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosbag2_interfaces::srv::TogglePaused_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosbag2_interfaces::srv::TogglePaused_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosbag2_interfaces::srv::TogglePaused_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosbag2_interfaces__srv__TogglePaused_Response
    std::shared_ptr<rosbag2_interfaces::srv::TogglePaused_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosbag2_interfaces__srv__TogglePaused_Response
    std::shared_ptr<rosbag2_interfaces::srv::TogglePaused_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TogglePaused_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const TogglePaused_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TogglePaused_Response_

// alias to use template instance with default allocator
using TogglePaused_Response =
  rosbag2_interfaces::srv::TogglePaused_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rosbag2_interfaces

namespace rosbag2_interfaces
{

namespace srv
{

struct TogglePaused
{
  using Request = rosbag2_interfaces::srv::TogglePaused_Request;
  using Response = rosbag2_interfaces::srv::TogglePaused_Response;
};

}  // namespace srv

}  // namespace rosbag2_interfaces

#endif  // ROSBAG2_INTERFACES__SRV__DETAIL__TOGGLE_PAUSED__STRUCT_HPP_
