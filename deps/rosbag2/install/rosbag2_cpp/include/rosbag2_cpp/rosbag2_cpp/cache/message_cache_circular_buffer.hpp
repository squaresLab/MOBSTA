// Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef ROSBAG2_CPP__CACHE__MESSAGE_CACHE_CIRCULAR_BUFFER_HPP_
#define ROSBAG2_CPP__CACHE__MESSAGE_CACHE_CIRCULAR_BUFFER_HPP_

#include <deque>
#include <memory>
#include <vector>

#include "rosbag2_cpp/visibility_control.hpp"
#include "rosbag2_cpp/cache/cache_buffer_interface.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"

// This is necessary because of using stl types here. It is completely safe, because
// a) the member is not accessible from the outside
// b) there are no inline functions.
#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable:4251)
#endif

namespace rosbag2_cpp
{
namespace cache
{

/**
* This class implements a circular buffer message cache. Since the buffer
* size is limited by total byte size of the storage messages rather than
* a fix number of messages, a deque is used instead of a vector since
* older messages can always be dropped from the front and new messages added
* to the end. The buffer will never consume more than max_cache_size bytes,
* and will log a warning message if an individual message exceeds the buffer
* size.
*/
class ROSBAG2_CPP_PUBLIC MessageCacheCircularBuffer
  : public CacheBufferInterface
{
public:
  // Delete default constructor since max_cache_size is required
  MessageCacheCircularBuffer() = delete;
  explicit MessageCacheCircularBuffer(size_t max_cache_size);

  /**
  * If buffer size has some space left, we push the message regardless of its size,
  *  but if this results in exceeding buffer size, we begin dropping old messages.
  */
  bool push(CacheBufferInterface::buffer_element_t msg) override;

  /// Clear buffer
  void clear() override;

  /// Get number of elements in the buffer
  size_t size() override;

  /// Get buffer data
  const std::vector<CacheBufferInterface::buffer_element_t> & data() override;

private:
  std::deque<CacheBufferInterface::buffer_element_t> buffer_;
  std::vector<CacheBufferInterface::buffer_element_t> msg_vector_;
  size_t buffer_bytes_size_ {0u};
  const size_t max_bytes_size_;
};

}  // namespace cache
}  // namespace rosbag2_cpp

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // ROSBAG2_CPP__CACHE__MESSAGE_CACHE_CIRCULAR_BUFFER_HPP_
