// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rosbag2_interfaces:msg/WriteSplitEvent.idl
// generated code does not contain a copyright notice

#ifndef ROSBAG2_INTERFACES__MSG__DETAIL__WRITE_SPLIT_EVENT__STRUCT_HPP_
#define ROSBAG2_INTERFACES__MSG__DETAIL__WRITE_SPLIT_EVENT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rosbag2_interfaces__msg__WriteSplitEvent __attribute__((deprecated))
#else
# define DEPRECATED__rosbag2_interfaces__msg__WriteSplitEvent __declspec(deprecated)
#endif

namespace rosbag2_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WriteSplitEvent_
{
  using Type = WriteSplitEvent_<ContainerAllocator>;

  explicit WriteSplitEvent_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->closed_file = "";
      this->opened_file = "";
    }
  }

  explicit WriteSplitEvent_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : closed_file(_alloc),
    opened_file(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->closed_file = "";
      this->opened_file = "";
    }
  }

  // field types and members
  using _closed_file_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _closed_file_type closed_file;
  using _opened_file_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _opened_file_type opened_file;

  // setters for named parameter idiom
  Type & set__closed_file(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->closed_file = _arg;
    return *this;
  }
  Type & set__opened_file(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->opened_file = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rosbag2_interfaces::msg::WriteSplitEvent_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosbag2_interfaces::msg::WriteSplitEvent_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosbag2_interfaces::msg::WriteSplitEvent_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosbag2_interfaces::msg::WriteSplitEvent_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosbag2_interfaces::msg::WriteSplitEvent_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosbag2_interfaces::msg::WriteSplitEvent_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosbag2_interfaces::msg::WriteSplitEvent_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosbag2_interfaces::msg::WriteSplitEvent_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosbag2_interfaces::msg::WriteSplitEvent_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosbag2_interfaces::msg::WriteSplitEvent_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosbag2_interfaces__msg__WriteSplitEvent
    std::shared_ptr<rosbag2_interfaces::msg::WriteSplitEvent_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosbag2_interfaces__msg__WriteSplitEvent
    std::shared_ptr<rosbag2_interfaces::msg::WriteSplitEvent_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WriteSplitEvent_ & other) const
  {
    if (this->closed_file != other.closed_file) {
      return false;
    }
    if (this->opened_file != other.opened_file) {
      return false;
    }
    return true;
  }
  bool operator!=(const WriteSplitEvent_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WriteSplitEvent_

// alias to use template instance with default allocator
using WriteSplitEvent =
  rosbag2_interfaces::msg::WriteSplitEvent_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rosbag2_interfaces

#endif  // ROSBAG2_INTERFACES__MSG__DETAIL__WRITE_SPLIT_EVENT__STRUCT_HPP_
