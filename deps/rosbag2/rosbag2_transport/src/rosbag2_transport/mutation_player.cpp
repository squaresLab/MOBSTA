/****
 * © 2023 Carnegie Mellon University. All rights reserved.
 * National Robotics Engineering Center, Carnegie Mellon University
 * www.nrec.ri.cmu.edu
 * Confidential and Proprietary - Do not distribute without prior written permission.
 *
 * License Status: Not Released.
 * (License Status to be confirmed by CTTEC prior to release from NREC)
 * This notice must appear in all copies of this file and its derivatives.
 ****/

/****
 * NREC Internal Use (Use as Background IP to be cleared by CTTEC and the Project Manager/PI prior to use on another project).
 * Created for Program: HPSTA - 55435.1.1990813
 ****/

#include <iostream>
#include <gorgon/mutators/base.h>
#include <gorgon/utils.h>
#include <fstream>
#include <boost/functional/hash.hpp>

#include "rosbag2_transport/mutation_player.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosbag2_cpp/types/introspection_message.hpp"
#include "rosbag2_cpp/serialization_format_converter_factory.hpp"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"

/**
 * @brief Hash function used to generate a per-message seed used in deterministic
 * mutational replay. Utilizes the djb2 algorithm for simplicity and consistency,
 * since std::hash is not deterministic between runs. 5381 and 33 are magic numbers
 * mainly used for statistically okay hashing values and are standard for djb2.
 * see http://www.cse.yorku.ca/~oz/hash.html
 *
 */
unsigned long hashDjb2(const uint8_t *input, const size_t input_size)
{
  unsigned long hash = 5381;
  for (size_t i = 0; i < input_size; i++)
  {
    hash += (hash << 5) + static_cast<unsigned long>(input[i]); /* hash*33 + c */
  }
  return hash;
}

namespace rosbag2_transport
{
  std::list<std::string> split_string(std::string s, const std::string &delimiter)
  {
    std::list<std::string> result;
    size_t pos = 0;
    // trim the leading slash
    s.erase(0, delimiter.length());
    std::string token;
    while ((pos = s.find(delimiter)) != std::string::npos)
    {
      token = s.substr(0, pos);
      result.push_back(token);
      s.erase(0, pos + delimiter.length());
    }
    result.push_back(s);
    return result;
  }

  size_t get_addr_from_member(const MemberInfo &member_info, std::list<std::string>::iterator &intercept_list_it, const std::list<std::string>::iterator &intercept_list_end, uint8_t* pointer_to_bytes)
  {
    if (intercept_list_it == intercept_list_end)
    {
      return -1;
    }

    // check array type
    if (member_info.is_array_)
    {
      // TODO handle arrays of non-primitives
      if (member_info.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE)
      {
        return -1;
      }


      // get the index if there is one
      ++intercept_list_it;
      size_t index = 0;
      if (intercept_list_it != intercept_list_end)
      {
        index = (size_t) std::stoi(intercept_list_it->c_str());
      }

      void* (*getter)(void*, size_t);
      getter = reinterpret_cast<void*(*)(void*, size_t)>(member_info.get_function);

      size_t pointer_to_field = (size_t)((*getter)(pointer_to_bytes + member_info.offset_, index));

      return pointer_to_field;
    }

    // check compsite message
    if (member_info.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE)
    {
      const auto &type_info = reinterpret_cast<const TypeInfo *>(member_info.members_->data);
      return get_addr_from_typeinfo(type_info, ++intercept_list_it, intercept_list_end, pointer_to_bytes + member_info.offset_);
    }

    // check by value
    if (std::strcmp(member_info.name_, intercept_list_it->c_str()) == 0)
    {

      if (++intercept_list_it != intercept_list_end)
        return -1;
      return member_info.offset_ + (size_t) pointer_to_bytes;
    }
    return -1;
  }

  size_t get_addr_from_typeinfo(const TypeInfo *type_info, std::list<std::string>::iterator intercept_list_it, const std::list<std::string>::iterator &intercept_list_end, uint8_t* pointer_to_bytes)
  {
    if (intercept_list_it == intercept_list_end)
      return -1;

    for (uint32_t i = 0; i < type_info->member_count_; ++i)
    {
      const MemberInfo &member_info = type_info->members_[i];
      if (std::strcmp(member_info.name_, intercept_list_it->c_str()) == 0)
      {
        size_t addr = get_addr_from_member(member_info, intercept_list_it, intercept_list_end, pointer_to_bytes);

        // if (member_info.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE)
        //   return offset + member_info.offset_;
        return addr;
      }
    }
    return -1;
  }


  MutationPlayer::MutationPlayer(
      std::unique_ptr<rosbag2_cpp::Reader> &&reader,
      const rosbag2_storage::StorageOptions &storage_options,
      const rosbag2_transport::PlayOptions &play_options,
      const std::string &mutation_config_file,
      const std::string &node_name,
      const rclcpp::NodeOptions &node_options)
      : Player(std::move(reader), storage_options, play_options, node_name, node_options),
        m_clock(),
        m_begin_time(m_clock.now()),
        m_rngDistribution(0.0, 1.0)
  {
    RCLCPP_INFO_ONCE(this->get_logger(), "log seed is : %ld", play_options.seed);

    // get the topics information to extract the message type
    rosbag2_storage::MetadataIo metadata_io;
    m_bag_metadata = metadata_io.read_metadata(storage_options.uri);

    // parse json file
    std::ifstream f(mutation_config_file);
    m_config_json = json::parse(f);

    json topics = m_config_json["topics"];
    utils::MutatorFactory factory;
    unsigned long log_seed = play_options.seed;
    m_rngEngine.seed(log_seed);

    for (json::iterator topic_it = topics.begin(); topic_it != topics.end(); ++topic_it)
    {
      const std::string &topic_of_interest = topic_it.key();

      m_topics.push_back(topic_of_interest);

      for (const auto &intercept_options : topic_it.value())
      {
        // calculate full intercept for message parser
        const std::string &intercept_of_interest = intercept_options["message_intercept"];
        m_intercept_map[intercept_of_interest] = split_string(intercept_of_interest, "/");
        std::string full_intercept = topic_of_interest + intercept_of_interest;

        // calculate per-field constructor seed to avoid copied mutations on different topics
        size_t intercept_hash = std::hash<std::string>{}(full_intercept);
        unsigned long mutator_seed = intercept_hash + log_seed;

        // parse config, make mutators, and add them to the topic's list of mutators
        for (const auto &mutator_description : intercept_options["mutations"])
        {
          json mutation_args = mutator_description["mutation_args"];
          std::string mutation_type = mutator_description["mutation_type"];

          m_mutators[full_intercept].push_back(factory.buildMutator(mutation_type, mutation_args, mutator_seed));
        }
      }
    }
  }

  MutationPlayer::~MutationPlayer()
  {

    for (auto cb_handle : keyboard_callbacks_)
    {
      keyboard_handler_->delete_key_press_callback(cb_handle);
    }
    // closes reader
    std::lock_guard<std::mutex> lk(reader_mutex_);
    if (reader_)
    {
      reader_->close();
    }
  }

  bool MutationPlayer::publish_message(rosbag2_storage::SerializedBagMessageSharedPtr message)
  {
    // determine if topic will be mutated
    if (std::find(m_topics.begin(), m_topics.end(), message->topic_name) == m_topics.end())
    {
      return Player::publish_message(message);
    }

    const double current_time = (m_clock.now() - m_begin_time).seconds();
    const std::string &topic_of_interest = message->topic_name;

    // extract the data type the topic uses
    auto &topics_metadata = m_bag_metadata.topics_with_message_count;
    auto topic_info_cmp = [topic_of_interest](const rosbag2_storage::TopicInformation &topic_info) -> bool
    { return topic_info.topic_metadata.name == topic_of_interest; };
    auto topic_it = std::find_if(topics_metadata.begin(), topics_metadata.end(), topic_info_cmp);

    if (topic_it == topics_metadata.end())
    {
      RCUTILS_LOG_ERROR_NAMED("mutation_player", "failed to find topic of name in the metadata.yaml file: %s", topic_of_interest.c_str());
    }
    std::string topic_type = topic_it->topic_metadata.type;

    // extract msg type information to prepare for deserialization
    InterfaceTypeName topic_type_name = {
        topic_type.substr(0, topic_type.find('/')),
        topic_type.substr(topic_type.rfind('/') + 1)};
    auto type_info = get_type_info(topic_type_name);
    const rosidl_message_type_support_t *ts = get_type_support(topic_type_name);

    auto deserializer = rclcpp::SerializationBase(ts);

    // create a buffer to handle the seralialization of the whole message
    auto msg_size = type_info->size_of_;

    std::shared_ptr<uint8_t> msg_buffer(new uint8_t[msg_size], [](uint8_t *ptr)
                                        { delete[] ptr; });
    std::memset(msg_buffer.get(), 0, type_info->size_of_);

    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>(*message->serialized_data);
    deserializer.deserialize_message(serialized_msg.get(), msg_buffer.get());

    unsigned long msg_seed = hashDjb2(msg_buffer.get(), msg_size);

    json topic_options = m_config_json["topics"][topic_of_interest];

    // find the location of each intercept and mutate it
    for (const json &intercept_options : topic_options)
    {
      // find data offset in buffer
      const std::string intercept_str = intercept_options.at("message_intercept").get<std::string>();
      std::string full_intercept = topic_of_interest + intercept_str;

      std::list<std::string> &intercept_list = m_intercept_map[intercept_str];

      size_t data_addr = get_addr_from_typeinfo(type_info, intercept_list.begin(), intercept_list.end(), msg_buffer.get());
      // RCUTILS_LOG_ERROR_NAMED("mutation_player", "offset %i", data_addr);
      if (data_addr == -1)
      {
        RCUTILS_LOG_ERROR_NAMED("mutation_player", "failed to find intercept of of name %s in definition of %s", intercept_str.c_str(), topic_type.c_str());
      }

      uint8_t *data = (uint8_t*) data_addr; // msg_buffer.get() + data_offset;

      const json &mutations = intercept_options["mutations"];
      for (size_t i = 0; i < mutations.size(); i++)
      {
        const auto &mutator_description = mutations[i];
        const float timeframe_begin = mutator_description["timeframe_begin"];
        const float timeframe_end = mutator_description["timeframe_end"];
        const float mutation_chance = mutator_description["mutation_chance"];
        if (current_time >= timeframe_begin && (timeframe_end == -1 || current_time < timeframe_end))
        {
          bool apply_mutation = m_rngDistribution(m_rngEngine) < mutation_chance;
          if (apply_mutation)
          {
            m_mutators[full_intercept][i]->mutateByReferenceSeeded(data, msg_seed);
          }
        }
      }
    }

    deserializer.serialize_message(msg_buffer.get(), serialized_msg.get());

    // repackage seraialized bag message to match function format
    auto serialized_data = std::make_shared<rcutils_uint8_array_t>(serialized_msg->get_rcl_serialized_message());
    rosbag2_storage::SerializedBagMessage bag_message = {
        serialized_data,
        message->time_stamp,
        message->topic_name};

    auto bag_message_ptr = std::make_shared<rosbag2_storage::SerializedBagMessage>(bag_message);

    return Player::publish_message(bag_message_ptr);
  }
}