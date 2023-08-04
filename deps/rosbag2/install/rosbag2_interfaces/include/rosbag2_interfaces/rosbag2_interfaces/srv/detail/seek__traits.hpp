// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosbag2_interfaces:srv/Seek.idl
// generated code does not contain a copyright notice

#ifndef ROSBAG2_INTERFACES__SRV__DETAIL__SEEK__TRAITS_HPP_
#define ROSBAG2_INTERFACES__SRV__DETAIL__SEEK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosbag2_interfaces/srv/detail/seek__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'time'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace rosbag2_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Seek_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: time
  {
    out << "time: ";
    to_flow_style_yaml(msg.time, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Seek_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time:\n";
    to_block_style_yaml(msg.time, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Seek_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace rosbag2_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use rosbag2_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rosbag2_interfaces::srv::Seek_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosbag2_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosbag2_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const rosbag2_interfaces::srv::Seek_Request & msg)
{
  return rosbag2_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rosbag2_interfaces::srv::Seek_Request>()
{
  return "rosbag2_interfaces::srv::Seek_Request";
}

template<>
inline const char * name<rosbag2_interfaces::srv::Seek_Request>()
{
  return "rosbag2_interfaces/srv/Seek_Request";
}

template<>
struct has_fixed_size<rosbag2_interfaces::srv::Seek_Request>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<rosbag2_interfaces::srv::Seek_Request>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<rosbag2_interfaces::srv::Seek_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosbag2_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Seek_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Seek_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Seek_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace rosbag2_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use rosbag2_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rosbag2_interfaces::srv::Seek_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosbag2_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosbag2_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const rosbag2_interfaces::srv::Seek_Response & msg)
{
  return rosbag2_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rosbag2_interfaces::srv::Seek_Response>()
{
  return "rosbag2_interfaces::srv::Seek_Response";
}

template<>
inline const char * name<rosbag2_interfaces::srv::Seek_Response>()
{
  return "rosbag2_interfaces/srv/Seek_Response";
}

template<>
struct has_fixed_size<rosbag2_interfaces::srv::Seek_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rosbag2_interfaces::srv::Seek_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rosbag2_interfaces::srv::Seek_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rosbag2_interfaces::srv::Seek>()
{
  return "rosbag2_interfaces::srv::Seek";
}

template<>
inline const char * name<rosbag2_interfaces::srv::Seek>()
{
  return "rosbag2_interfaces/srv/Seek";
}

template<>
struct has_fixed_size<rosbag2_interfaces::srv::Seek>
  : std::integral_constant<
    bool,
    has_fixed_size<rosbag2_interfaces::srv::Seek_Request>::value &&
    has_fixed_size<rosbag2_interfaces::srv::Seek_Response>::value
  >
{
};

template<>
struct has_bounded_size<rosbag2_interfaces::srv::Seek>
  : std::integral_constant<
    bool,
    has_bounded_size<rosbag2_interfaces::srv::Seek_Request>::value &&
    has_bounded_size<rosbag2_interfaces::srv::Seek_Response>::value
  >
{
};

template<>
struct is_service<rosbag2_interfaces::srv::Seek>
  : std::true_type
{
};

template<>
struct is_service_request<rosbag2_interfaces::srv::Seek_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rosbag2_interfaces::srv::Seek_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROSBAG2_INTERFACES__SRV__DETAIL__SEEK__TRAITS_HPP_
