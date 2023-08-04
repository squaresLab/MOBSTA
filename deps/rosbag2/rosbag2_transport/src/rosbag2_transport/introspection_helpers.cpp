// Based on https://github.com/osrf/dynamic_message_introspection/blob/main/dynmsg/src/typesupport.cpp commit 4afd277
// https://github.com/osrf/dynamic_message_introspection/blob/main/dynmsg_demo/src/typesupport_utils.cpp commit 4afd277
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

#include <sstream>
#include <dlfcn.h>

#include "rcutils/logging_macros.h"
#include "rcutils/allocator.h"
#include "rosbag2_transport/introspection_helpers.hpp"

const TypeInfo *get_type_info(const InterfaceTypeName &interface_type)
{
    // Load the introspection library for the package containing the requested type
    std::stringstream ts_lib_name;
    ts_lib_name << "lib" << interface_type.first << "__rosidl_typesupport_introspection_c.so";
    RCUTILS_LOG_DEBUG_NAMED(
        "dynmsg",
        "Loading introspection type support library %s",
        ts_lib_name.str().c_str());
    void *introspection_type_support_lib = dlopen(ts_lib_name.str().c_str(), RTLD_LAZY);
    if (introspection_type_support_lib == nullptr)
    {
        RCUTILS_LOG_ERROR_NAMED(
            "dynmsg", "failed to load introspection type support library: %s", dlerror());
        return nullptr;
    }
    // Load the function that, when called, will give us the introspection information for the
    // interface type we are interested in
    std::stringstream ts_func_name;
    ts_func_name << "rosidl_typesupport_introspection_c__get_message_type_support_handle__" << interface_type.first << "__msg__" << interface_type.second;
    RCUTILS_LOG_DEBUG_NAMED(
        "dynmsg", "Loading type support function %s", ts_func_name.str().c_str());

    get_message_ts_func introspection_type_support_handle_func =
        reinterpret_cast<get_message_ts_func>(dlsym(
            introspection_type_support_lib,
            ts_func_name.str().c_str()));
    if (introspection_type_support_handle_func == nullptr)
    {
        RCUTILS_LOG_ERROR_NAMED(
            "dynmsg",
            "failed to load introspection type support function: %s",
            dlerror());
        return nullptr;
    }

    // Call the function to get the introspection information we want
    const rosidl_message_type_support_t *introspection_ts =
        introspection_type_support_handle_func();
    RCUTILS_LOG_DEBUG_NAMED(
        "dynmsg",
        "Loaded type support %s",
        introspection_ts->typesupport_identifier);
    const rosidl_typesupport_introspection_c__MessageMembers *type_info =
        reinterpret_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(
            introspection_ts->data);

    return type_info;
}

const TypeSupport *get_type_support(const InterfaceTypeName &interface_type)
{
    // Load the type support library for the package containing the requested type
    std::string ts_lib_name;
    ts_lib_name = "lib" + interface_type.first + "__rosidl_typesupport_c.so";
    RCUTILS_LOG_DEBUG_NAMED(
        "dynmsg_demo",
        "Loading type support library %s",
        ts_lib_name.c_str());
    void *type_support_lib = dlopen(ts_lib_name.c_str(), RTLD_LAZY);
    if (type_support_lib == nullptr)
    {
        RCUTILS_LOG_ERROR_NAMED("dynmsg_demo", "failed to load type support library: %s", dlerror());
        return nullptr;
    }
    // Load the function that, when called, will give us the type support for the interface type we
    // are interested in
    std::string ts_func_name;
    ts_func_name = "rosidl_typesupport_c__get_message_type_support_handle__" + interface_type.first +
                   "__msg__" + interface_type.second;
    RCUTILS_LOG_DEBUG_NAMED("dynmsg_demo", "Loading type support function %s", ts_func_name.c_str());

    get_message_ts_func type_support_handle_func =
        reinterpret_cast<get_message_ts_func>(dlsym(type_support_lib, ts_func_name.c_str()));
    if (type_support_handle_func == nullptr)
    {
        RCUTILS_LOG_ERROR_NAMED("dynmsg_demo", "failed to load type support function: %s", dlerror());
        return nullptr;
    }

    // Call the function to get the type support we want
    const rosidl_message_type_support_t *ts = type_support_handle_func();
    RCUTILS_LOG_DEBUG_NAMED("dynmsg_demo", "Loaded type support %s", ts->typesupport_identifier);

    return ts;
}