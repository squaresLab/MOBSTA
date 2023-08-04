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

#include "rosbag/mutation_player.h"
#include <cstdio>
#include <iostream>
#include <fstream>
#include <map>

#include <boost/functional/hash.hpp>

#define DEBUG 0


namespace rosbag{


MutationPlayer::
MutationPlayer(PlayerOptions const& options)
  : Player(options), 
    m_begin_time(ros::Time::now()),
    m_rngDistribution(0.0,1.0)
{
  options_.check();

  // preload bags in case pre-processing is needed
  Player::openBags(options);

  // load config file
  const std::string& config_file_name = options_.config_file;
  std::ifstream f(config_file_name);
  m_config_json = json::parse(f);

  json topics = m_config_json["topics"];
  utils::RosMutatorFactory factory;
  unsigned long log_seed;

  // Determine Log Seed (given or randomly generated)
  if (options_.has_seed) {
    log_seed = options_.seed;
  }
  else {
    log_seed = std::random_device()();
    ROS_INFO("Using Random Mutation Seed: %lu", log_seed);
  }
  m_rngEngine.seed(log_seed);

  // extract targeted topics for mutation and create mutators
  for(json::iterator topic_it = topics.begin(); topic_it != topics.end(); ++topic_it){

    const std::string& topic_of_interest = topic_it.key();

    for(const auto& intercept_options : topic_it.value()) {

      // calculate full intercept for message parser
      const std::string& intercept_of_interest = intercept_options["message_intercept"];
      std::string full_intercept = topic_of_interest + intercept_of_interest;

      m_intercept_map[topic_of_interest].push_back(full_intercept);

      // calculate per-field constructor seed to avoid copied mutations on different topics
      size_t intercept_hash = std::hash<std::string>{}(full_intercept);
      unsigned long mutator_seed = intercept_hash + log_seed;

      // parse config, make mutators, and add them to the topic's list of mutators
      for (const auto& mutator_description : intercept_options["mutations"]){
        json mutation_args = mutator_description["mutation_args"];
        std::string mutation_type = mutator_description["mutation_type"];

        m_Mutators[full_intercept].push_back(factory.buildMutator(mutation_type, mutation_args, mutator_seed));
      }
    }

  }

  // go through bags, find topics that will be mutated, and create parsers for them
  for(const auto& bag: bags_){
    View view(*bag);
    for(const auto& connection: view.getConnections()){
      const std::string& topic_name = connection->topic;
      const auto& topic_it = m_intercept_map.find(topic_name);

      if(topic_it == m_intercept_map.end()) continue;

      m_MessageParsers[topic_name] = std::make_unique<RosMsgParser::Parser>(topic_name, connection->datatype, connection->msg_def);
      m_MessageParsers[topic_name]->setMaxArrayPolicy(RosMsgParser::Parser::MaxArrayPolicy::KEEP_LARGE_ARRAYS, 100);
      m_MessageParsers[topic_name]->setBlobPolicy(RosMsgParser::Parser::BlobPolicy::STORE_BLOB_AS_REFERENCE);
    }
  }

}


MutationPlayer::
~MutationPlayer(){
}

void
MutationPlayer::
msg_publish(ros::Publisher& pub, rosbag::MessageInstance const& m) {
  std::string topic_name = m.getTopic();

  // determine if topic will be mutated
  if(!m_config_json["topics"].contains(topic_name)){
    pub.publish(m);
    return;
  }

  json topic_options = m_config_json["topics"][topic_name];

  const double current_time = (ros::Time::now() - m_begin_time).toSec();

  // convert MessageInstance to ShapeShifter to publish a generic message after mutation
  RosMsgParser::ShapeShifter generic_message;

  // load bag data into buffer
  boost::shared_array<uint8_t> buffer(new uint8_t[m.size()]);
  ros::serialization::OStream ostream(buffer.get(), m.size());
  m.write(ostream);

  // read buffer into new shapeshifter so it can be accessed
  ros::serialization::IStream istream(buffer.get(), m.size());
  generic_message.read(istream);

  // create a flat message to extract information
  RosMsgParser::Span<uint8_t> buffer_span(generic_message.raw_data(), generic_message.size());
  RosMsgParser::FlatMessage flat_container;
  m_MessageParsers[topic_name]->deserializeIntoFlatMsg(buffer_span, &flat_container);

  unsigned long msg_seed = boost::hash_range(generic_message.raw_data(), 
                                             generic_message.raw_data()+generic_message.size());

  // find the location of each intercept and mutate it
  for(const auto& intercept_options : topic_options) {
    std::string full_intercept = topic_name + intercept_options.at("message_intercept").get<std::string>();
    // setup for finding the intercept message
    auto* desired_data_begin = &(*flat_container.blob.begin()->second.data());
    size_t desired_data_size = 0;
    bool found_intercept = false;

    // find primitives that contain the goal intercept in their path 
    // (for ex: intercept '/path/poses.0' is a prefix  of '/path/poses.0/position/x')
    // record the first occurence of this and then add sizes until the occurence is done
    for (auto blob_iter = flat_container.blob.begin(); blob_iter != flat_container.blob.end(); blob_iter++){
      if (blob_iter->first.toStdString().find(full_intercept) == 0){

        if (!found_intercept){
          desired_data_begin = blob_iter->second.data();
        }
        
        found_intercept = true;

        desired_data_size += blob_iter->second.size();
      }
    }

    if(!found_intercept){
      // intercept not found, so publish base message.
      // might be mutating something that isnt there yet 
      // (for ex: 2nd index in a slowly filling array)
      ROS_WARN("Error: intercept field %s doesn't appear in the message. Publishing unmutated message", full_intercept.c_str());
      pub.publish(m);
      return;
    }

    // ROS saves pointer as immutable constant
    // Gorgon knows how to deal with serialized ros data
    // so this is safe
    auto* editable_data = const_cast<unsigned char*>(desired_data_begin);

    // get mutation variables described for this topic
    const json mutations = intercept_options["mutations"];

    // get mutation start/end/chance, then
    // apply each mutation to extracted data one at a time
    for(size_t i = 0; i < mutations.size(); i++){
      
      const auto& mutator_description = mutations[i];
      const float timeframe_begin = mutator_description["timeframe_begin"];
      const float timeframe_end = mutator_description["timeframe_end"];
      const float mutation_chance = mutator_description["mutation_chance"];

      if(current_time >= timeframe_begin && (timeframe_end == -1 || current_time < timeframe_end))
      {
        bool apply_mutation = m_rngDistribution(m_rngEngine) < mutation_chance;
        if (apply_mutation)
        {
          m_Mutators[full_intercept][i]->mutateSerializedSeeded(editable_data, desired_data_size, msg_seed);
        }
      }
      
    }
  }
  generic_message.morph(m.getMD5Sum(), m.getDataType(), m.getMessageDefinition());
  pub.publish(generic_message);
}
}
