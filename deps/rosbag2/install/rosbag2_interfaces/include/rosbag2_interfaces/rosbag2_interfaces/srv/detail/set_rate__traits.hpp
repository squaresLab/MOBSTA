// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosbag2_interfaces:srv/SetRate.idl
// generated code does not contain a copyright notice

#ifndef ROSBAG2_INTERFACES__SRV__DETAIL__SET_RATE__TRAITS_HPP_
#define ROSBAG2_INTERFACES__SRV__DETAIL__SET_RATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosbag2_interfaces/srv/detail/set_rate__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rosbag2_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetRate_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: rate
  {
    out << "rate: ";
    rosidl_generator_traits::value_to_yaml(msg.rate, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetRate_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rate: ";
    rosidl_generator_traits::value_to_yaml(msg.rate, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetRate_Request & msg, bool use_flow_style = false)
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
  const rosbag2_interfaces::srv::SetRate_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosbag2_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosbag2_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const rosbag2_interfaces::srv::SetRate_Request & msg)
{
  return rosbag2_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rosbag2_interfaces::srv::SetRate_Request>()
{
  return "rosbag2_interfaces::srv::SetRate_Request";
}

template<>
inline const char * name<rosbag2_interfaces::srv::SetRate_Request>()
{
  return "rosbag2_interfaces/srv/SetRate_Request";
}

template<>
struct has_fixed_size<rosbag2_interfaces::srv::SetRate_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rosbag2_interfaces::srv::SetRate_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rosbag2_interfaces::srv::SetRate_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosbag2_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetRate_Response & msg,
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
  const SetRate_Response & msg,
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

inline std::string to_yaml(const SetRate_Response & msg, bool use_flow_style = false)
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
  const rosbag2_interfaces::srv::SetRate_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosbag2_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosbag2_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const rosbag2_interfaces::srv::SetRate_Response & msg)
{
  return rosbag2_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rosbag2_interfaces::srv::SetRate_Response>()
{
  return "rosbag2_interfaces::srv::SetRate_Response";
}

template<>
inline const char * name<rosbag2_interfaces::srv::SetRate_Response>()
{
  return "rosbag2_interfaces/srv/SetRate_Response";
}

template<>
struct has_fixed_size<rosbag2_interfaces::srv::SetRate_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rosbag2_interfaces::srv::SetRate_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rosbag2_interfaces::srv::SetRate_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rosbag2_interfaces::srv::SetRate>()
{
  return "rosbag2_interfaces::srv::SetRate";
}

template<>
inline const char * name<rosbag2_interfaces::srv::SetRate>()
{
  return "rosbag2_interfaces/srv/SetRate";
}

template<>
struct has_fixed_size<rosbag2_interfaces::srv::SetRate>
  : std::integral_constant<
    bool,
    has_fixed_size<rosbag2_interfaces::srv::SetRate_Request>::value &&
    has_fixed_size<rosbag2_interfaces::srv::SetRate_Response>::value
  >
{
};

template<>
struct has_bounded_size<rosbag2_interfaces::srv::SetRate>
  : std::integral_constant<
    bool,
    has_bounded_size<rosbag2_interfaces::srv::SetRate_Request>::value &&
    has_bounded_size<rosbag2_interfaces::srv::SetRate_Response>::value
  >
{
};

template<>
struct is_service<rosbag2_interfaces::srv::SetRate>
  : std::true_type
{
};

template<>
struct is_service_request<rosbag2_interfaces::srv::SetRate_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rosbag2_interfaces::srv::SetRate_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROSBAG2_INTERFACES__SRV__DETAIL__SET_RATE__TRAITS_HPP_
