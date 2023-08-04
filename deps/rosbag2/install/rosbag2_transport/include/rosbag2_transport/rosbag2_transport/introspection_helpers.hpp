// Based on https://github.com/osrf/dynamic_message_introspection/blob/main/dynmsg/include/dynmsg/typesupport.hpp commit 4afd277
// Based on https://github.com/osrf/dynamic_message_introspection/blob/main/dynmsg_demo/include/dynmsg_demo/typesupport_utils.hpp commit 4afd277
//
// Original code used under Apache license:
// http://www.apache.org/licenses/LICENSE-2.0
//
// Copyright 2020 Open Source Robotics Foundation, Inc.
// Copyright 2021 Christophe Bedard
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <utility>
#include <string>

#include "rosidl_typesupport_introspection_c/message_introspection.h"

using InterfaceTypeName = std::pair<std::string, std::string>;
using TypeInfo_C = rosidl_typesupport_introspection_c__MessageMembers;
using MemberInfo_C = rosidl_typesupport_introspection_c__MessageMember;
using TypeInfo = TypeInfo_C;
using MemberInfo = MemberInfo_C;
using TypeSupport = rosidl_message_type_support_t;

typedef const rosidl_message_type_support_t *(*get_message_ts_func)();

/**
 * @brief provides a offset layout to the ROS message byte array
 *
 * @param interface_type a pair contains message package name and message type ex. {"std_msgs", "Int16"}
 */
const TypeInfo *get_type_info(const InterfaceTypeName &interface_type);

/**
 * @brief provides pointer to the message introspection data to be used for serialization/deserialization
 *
 * @param interface_type a pair contains message package name and message type ex. {"std_msgs", "Int16"}
 */
const TypeSupport *get_type_support(const InterfaceTypeName &interface_type);