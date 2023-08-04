// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rosbag2_interfaces:srv/Pause.idl
// generated code does not contain a copyright notice

#ifndef ROSBAG2_INTERFACES__SRV__DETAIL__PAUSE__STRUCT_HPP_
#define ROSBAG2_INTERFACES__SRV__DETAIL__PAUSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rosbag2_interfaces__srv__Pause_Request __attribute__((deprecated))
#else
# define DEPRECATED__rosbag2_interfaces__srv__Pause_Request __declspec(deprecated)
#endif

namespace rosbag2_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Pause_Request_
{
  using Type = Pause_Request_<ContainerAllocator>;

  explicit Pause_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit Pause_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    rosbag2_interfaces::srv::Pause_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosbag2_interfaces::srv::Pause_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosbag2_interfaces::srv::Pause_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosbag2_interfaces::srv::Pause_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosbag2_interfaces::srv::Pause_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosbag2_interfaces::srv::Pause_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosbag2_interfaces::srv::Pause_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosbag2_interfaces::srv::Pause_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosbag2_interfaces::srv::Pause_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosbag2_interfaces::srv::Pause_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosbag2_interfaces__srv__Pause_Request
    std::shared_ptr<rosbag2_interfaces::srv::Pause_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosbag2_interfaces__srv__Pause_Request
    std::shared_ptr<rosbag2_interfaces::srv::Pause_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Pause_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const Pause_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Pause_Request_

// alias to use template instance with default allocator
using Pause_Request =
  rosbag2_interfaces::srv::Pause_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rosbag2_interfaces


#ifndef _WIN32
# define DEPRECATED__rosbag2_interfaces__srv__Pause_Response __attribute__((deprecated))
#else
# define DEPRECATED__rosbag2_interfaces__srv__Pause_Response __declspec(deprecated)
#endif

namespace rosbag2_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Pause_Response_
{
  using Type = Pause_Response_<ContainerAllocator>;

  explicit Pause_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit Pause_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    rosbag2_interfaces::srv::Pause_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosbag2_interfaces::srv::Pause_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosbag2_interfaces::srv::Pause_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosbag2_interfaces::srv::Pause_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosbag2_interfaces::srv::Pause_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosbag2_interfaces::srv::Pause_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosbag2_interfaces::srv::Pause_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosbag2_interfaces::srv::Pause_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosbag2_interfaces::srv::Pause_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosbag2_interfaces::srv::Pause_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosbag2_interfaces__srv__Pause_Response
    std::shared_ptr<rosbag2_interfaces::srv::Pause_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosbag2_interfaces__srv__Pause_Response
    std::shared_ptr<rosbag2_interfaces::srv::Pause_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Pause_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const Pause_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Pause_Response_

// alias to use template instance with default allocator
using Pause_Response =
  rosbag2_interfaces::srv::Pause_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rosbag2_interfaces

namespace rosbag2_interfaces
{

namespace srv
{

struct Pause
{
  using Request = rosbag2_interfaces::srv::Pause_Request;
  using Response = rosbag2_interfaces::srv::Pause_Response;
};

}  // namespace srv

}  // namespace rosbag2_interfaces

#endif  // ROSBAG2_INTERFACES__SRV__DETAIL__PAUSE__STRUCT_HPP_
