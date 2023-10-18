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

#include <deque>
#include <memory>
#include <vector>

#include "rosbag2_cpp/logging.hpp"
#include "rosbag2_cpp/cache/cache_buffer_interface.hpp"
#include "rosbag2_cpp/cache/message_cache_circular_buffer.hpp"

namespace rosbag2_cpp
{
namespace cache
{

MessageCacheCircularBuffer::MessageCacheCircularBuffer(size_t max_cache_size)
: max_bytes_size_(max_cache_size)
{
}

bool MessageCacheCircularBuffer::push(CacheBufferInterface::buffer_element_t msg)
{
  // Drop message if it exceeds the buffer size
  if (msg->serialized_data->buffer_length > max_bytes_size_) {
    ROSBAG2_CPP_LOG_WARN_STREAM("Last message exceeds snapshot buffer size. Dropping message!");
    return false;
  }

  // Remove any old items until there is room for new message
  while (buffer_bytes_size_ > (max_bytes_size_ - msg->serialized_data->buffer_length)) {
    buffer_bytes_size_ -= buffer_.front()->serialized_data->buffer_length;
    buffer_.pop_front();
  }
  // Add new message to end of buffer
  buffer_bytes_size_ += msg->serialized_data->buffer_length;
  buffer_.push_back(msg);

  return true;
}

void MessageCacheCircularBuffer::clear()
{
  buffer_.clear();
  buffer_bytes_size_ = 0u;
}

size_t MessageCacheCircularBuffer::size()
{
  return buffer_.size();
}

const std::vector<CacheBufferInterface::buffer_element_t> & MessageCacheCircularBuffer::data()
{
  // Copy data to vector to maintain same interface as MessageCacheBuffer
  msg_vector_ = std::vector<CacheBufferInterface::buffer_element_t>(
    buffer_.begin(), buffer_.end());
  return msg_vector_;
}

}  // namespace cache
}  // namespace rosbag2_cpp
