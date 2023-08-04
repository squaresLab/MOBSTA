// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosbag2_interfaces:srv/IsPaused.idl
// generated code does not contain a copyright notice

#ifndef ROSBAG2_INTERFACES__SRV__DETAIL__IS_PAUSED__TRAITS_HPP_
#define ROSBAG2_INTERFACES__SRV__DETAIL__IS_PAUSED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosbag2_interfaces/srv/detail/is_paused__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rosbag2_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const IsPaused_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const IsPaused_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const IsPaused_Request & msg, bool use_flow_style = false)
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
  const rosbag2_interfaces::srv::IsPaused_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosbag2_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosbag2_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const rosbag2_interfaces::srv::IsPaused_Request & msg)
{
  return rosbag2_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rosbag2_interfaces::srv::IsPaused_Request>()
{
  return "rosbag2_interfaces::srv::IsPaused_Request";
}

template<>
inline const char * name<rosbag2_interfaces::srv::IsPaused_Request>()
{
  return "rosbag2_interfaces/srv/IsPaused_Request";
}

template<>
struct has_fixed_size<rosbag2_interfaces::srv::IsPaused_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rosbag2_interfaces::srv::IsPaused_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rosbag2_interfaces::srv::IsPaused_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosbag2_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const IsPaused_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: paused
  {
    out << "paused: ";
    rosidl_generator_traits::value_to_yaml(msg.paused, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const IsPaused_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: paused
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "paused: ";
    rosidl_generator_traits::value_to_yaml(msg.paused, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const IsPaused_Response & msg, bool use_flow_style = false)
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
  const rosbag2_interfaces::srv::IsPaused_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosbag2_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosbag2_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const rosbag2_interfaces::srv::IsPaused_Response & msg)
{
  return rosbag2_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rosbag2_interfaces::srv::IsPaused_Response>()
{
  return "rosbag2_interfaces::srv::IsPaused_Response";
}

template<>
inline const char * name<rosbag2_interfaces::srv::IsPaused_Response>()
{
  return "rosbag2_interfaces/srv/IsPaused_Response";
}

template<>
struct has_fixed_size<rosbag2_interfaces::srv::IsPaused_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rosbag2_interfaces::srv::IsPaused_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rosbag2_interfaces::srv::IsPaused_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rosbag2_interfaces::srv::IsPaused>()
{
  return "rosbag2_interfaces::srv::IsPaused";
}

template<>
inline const char * name<rosbag2_interfaces::srv::IsPaused>()
{
  return "rosbag2_interfaces/srv/IsPaused";
}

template<>
struct has_fixed_size<rosbag2_interfaces::srv::IsPaused>
  : std::integral_constant<
    bool,
    has_fixed_size<rosbag2_interfaces::srv::IsPaused_Request>::value &&
    has_fixed_size<rosbag2_interfaces::srv::IsPaused_Response>::value
  >
{
};

template<>
struct has_bounded_size<rosbag2_interfaces::srv::IsPaused>
  : std::integral_constant<
    bool,
    has_bounded_size<rosbag2_interfaces::srv::IsPaused_Request>::value &&
    has_bounded_size<rosbag2_interfaces::srv::IsPaused_Response>::value
  >
{
};

template<>
struct is_service<rosbag2_interfaces::srv::IsPaused>
  : std::true_type
{
};

template<>
struct is_service_request<rosbag2_interfaces::srv::IsPaused_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rosbag2_interfaces::srv::IsPaused_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROSBAG2_INTERFACES__SRV__DETAIL__IS_PAUSED__TRAITS_HPP_
