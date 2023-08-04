// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosbag2_interfaces:msg/WriteSplitEvent.idl
// generated code does not contain a copyright notice

#ifndef ROSBAG2_INTERFACES__MSG__DETAIL__WRITE_SPLIT_EVENT__TRAITS_HPP_
#define ROSBAG2_INTERFACES__MSG__DETAIL__WRITE_SPLIT_EVENT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosbag2_interfaces/msg/detail/write_split_event__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rosbag2_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const WriteSplitEvent & msg,
  std::ostream & out)
{
  out << "{";
  // member: closed_file
  {
    out << "closed_file: ";
    rosidl_generator_traits::value_to_yaml(msg.closed_file, out);
    out << ", ";
  }

  // member: opened_file
  {
    out << "opened_file: ";
    rosidl_generator_traits::value_to_yaml(msg.opened_file, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WriteSplitEvent & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: closed_file
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "closed_file: ";
    rosidl_generator_traits::value_to_yaml(msg.closed_file, out);
    out << "\n";
  }

  // member: opened_file
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "opened_file: ";
    rosidl_generator_traits::value_to_yaml(msg.opened_file, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WriteSplitEvent & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace rosbag2_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use rosbag2_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rosbag2_interfaces::msg::WriteSplitEvent & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosbag2_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosbag2_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const rosbag2_interfaces::msg::WriteSplitEvent & msg)
{
  return rosbag2_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rosbag2_interfaces::msg::WriteSplitEvent>()
{
  return "rosbag2_interfaces::msg::WriteSplitEvent";
}

template<>
inline const char * name<rosbag2_interfaces::msg::WriteSplitEvent>()
{
  return "rosbag2_interfaces/msg/WriteSplitEvent";
}

template<>
struct has_fixed_size<rosbag2_interfaces::msg::WriteSplitEvent>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rosbag2_interfaces::msg::WriteSplitEvent>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rosbag2_interfaces::msg::WriteSplitEvent>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROSBAG2_INTERFACES__MSG__DETAIL__WRITE_SPLIT_EVENT__TRAITS_HPP_
