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

#ifndef ROSBAG2_TRANSPORT__MUTATION_PLAYER_HPP_
#define ROSBAG2_TRANSPORT__MUTATION_PLAYER_HPP_

#include "rosbag2_transport/player.hpp"
#include "rosbag2_transport/play_options.hpp"
#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_transport/introspection_helpers.hpp"
#include <gorgon/mutators/base.h>
#include <nlohmann/json.hpp>
#include <random>

using json = nlohmann::json;

namespace rosbag2_transport
{
  size_t get_addr_from_typeinfo(const TypeInfo *type_info, std::list<std::string>::iterator intercept_list_it, const std::list<std::string>::iterator &intercept_list_end, uint8_t* pointer_to_bytes);

  class MutationPlayer : public Player
  {
  public:
    /**
     * @brief Constructs class with The same funciton of the parent class while also
     *     adding ways to parse the messages arbitrarily. it also uses the metadata.yaml
     *     file to get the types published on the respective topics
     *
     * @param reader opens bags and reads messages from the bag file
     * @param storage_options information about the bag
     * @param play_options command line options of how to publish
     * @param node_name name of the player node
     * @param mutation_config_file file name of the JSON file that configures
     *     the mutation topic and time ranges.
     */
    ROSBAG2_TRANSPORT_PUBLIC
    MutationPlayer(
        std::unique_ptr<rosbag2_cpp::Reader> &&reader,
        const rosbag2_storage::StorageOptions &storage_options,
        const rosbag2_transport::PlayOptions &play_options,
        const std::string &mutation_config_file = "",
        const std::string &node_name = "rosbag2_mutation_player",
        const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());

    virtual ~MutationPlayer();

    /**
     * @brief mutates a bag messages at the time of publishing. It relies on the parent
     *      publishing method .
     *
     * @param message the serialized message to be published
     */
    virtual bool publish_message(rosbag2_storage::SerializedBagMessageSharedPtr message);

  private:
    rclcpp::Clock m_clock;

    /**
     * @brief the time the player started
     */
    rclcpp::Time m_begin_time;

    /**
     * @private
     *
     * @brief distribution for deciding whether or not to mutate the data
     */
    std::uniform_real_distribution<> m_rngDistribution;

    /**
     * @private
     *
     * @brief Mapping from an topic to collection of Mutator Object ptrs
     */
    std::map<std::string, std::vector<std::shared_ptr<BaseMutator>>> m_mutators;

    /**
     * @private
     *
     * @brief The random number generator used in deciding whether or not to mutate the data
     */
    std::mt19937 m_rngEngine;

    /**
     * @brief information about the bag topics and message types
     */
    rosbag2_storage::BagMetadata m_bag_metadata;

    /**
     * @brief names of topics to be mutated.
     */
    std::vector<std::string> m_topics;

    /**
     * @brief configurations of the mutation topics and time ranges of their application.
     */
    json m_config_json;

     /**
     * @brief stores preprocessed split of the intercept to save time during publishing.
     */
    std::map<std::string, std::list<std::string>> m_intercept_map;

  };
}

#endif // ROSBAG2_TRANSPORT__MUTATION_PLAYER_HPP_