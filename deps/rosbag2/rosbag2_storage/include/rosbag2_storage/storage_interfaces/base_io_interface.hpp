// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef ROSBAG2_STORAGE__STORAGE_INTERFACES__BASE_IO_INTERFACE_HPP_
#define ROSBAG2_STORAGE__STORAGE_INTERFACES__BASE_IO_INTERFACE_HPP_

#include <string>

#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/visibility_control.hpp"

namespace rosbag2_storage
{
namespace storage_interfaces
{

enum class IOFlag : uint8_t
{
  READ_ONLY = 0,
  READ_WRITE = 1,
  APPEND = 2
};

// When bagfile splitting feature is not enabled or applicable,
// use 0 as the default maximum bagfile size value.
ROSBAG2_STORAGE_PUBLIC extern const uint64_t MAX_BAGFILE_SIZE_NO_SPLIT;

// When bagfile splitting feature is not enabled or applicable,
// use 0 as the default maximum bagfile duration value.
ROSBAG2_STORAGE_PUBLIC extern const uint64_t MAX_BAGFILE_DURATION_NO_SPLIT;

class ROSBAG2_STORAGE_PUBLIC BaseIOInterface
{
public:
  virtual ~BaseIOInterface() = default;

  /**
   * Opens the storage plugin.
   * \param storage_options contains necessary info such as file uri and storage plugin.
   * \param io_flag is a hint for the type of storage plugin to open depending on the io operations requested.
   * \param config_file_uri is a path to a storage specific file path.
   * If IOFlag::READ_ONLY is passed, then only read operations are guaranteed.
   * The uri passed should be the exact relative path to the bagfile.
   * If IOFlag::READ_WRITE is passed, then a new bagfile is created with guaranteed read and write operations.
   * The storage plugin will append the uri in the case of creating a new bagfile backing.
   * The storage can load specific configurations by accessing a config file given in the `config_file_uri` parameter.
   */
  virtual void open(
    const StorageOptions & storage_options,
    IOFlag io_flag) = 0;
};

}  // namespace storage_interfaces
}  // namespace rosbag2_storage

#endif  // ROSBAG2_STORAGE__STORAGE_INTERFACES__BASE_IO_INTERFACE_HPP_
