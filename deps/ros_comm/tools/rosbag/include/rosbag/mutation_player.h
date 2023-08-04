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


#ifndef __ROSBAGMUTATIONPLAYER_H__
#define __ROSBAGMUTATIONPLAYER_H__

#include <vector>
#include <random>

#include "gorgon_ros/mutators/ros_base.h"
#include "gorgon_ros/utils.h"
#include "player.h"
#include "ros_msg_parser/ros_parser.hpp"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace rosbag{

    /**
     * @brief Player class able to handle custom mutations at publish time.
     *
     * @note This is a child of the Player class with inheriting its publishing functionality.
     */
    class MutationPlayer: public Player{
        public:
            /**
             * @brief Constructs class with The same funciton of the parent class while also 
             *     adding ways to parse the messages arbitrarily
             *
             * @param options The options provided to the player through the terminal
             */
            MutationPlayer(PlayerOptions const& options);

            ~MutationPlayer();

        private:
            /**
             * @brief wrapper around the publishers publish to allow the replay to mutate the messages before publishing
             *
             * @param pub A ros publisher to publish the message to the appropriate topic.
             * @param m   A Message Instance which allows rosbag to work with arbitrary messages.
             */
            void msg_publish(ros::Publisher& pub, rosbag::MessageInstance const& m) override;

        private:
            /**
             * @private
             *
             * @brief Map from topic to parser that allows for a way to do introspection on a msg. 
             */
            std::map<std::string, std::unique_ptr<RosMsgParser::Parser>> m_MessageParsers;

            /**
             * @private
             *
             * @brief Specifies a map of the fields in a given message to be mutated.
             */
            std::map<std::string, std::vector<std::string>> m_intercept_map;

            /**
             * @private
             *
             * @brief JSON object that has the list of fields intercepts and their respective mutation list.
             */
            json m_config_json;

            /**
             * @private
             *
             * @brief Record start time of the program to effectively apply mutations at the specified time ranges.
             */
            ros::Time m_begin_time;

            /**
             * @private
             * 
             * @brief Mapping from an intercept to collection of Mutator Object ptrs
             */
            std::map<std::string, std::vector<std::shared_ptr<RosBaseMutator>> > m_Mutators;


            /**
             * @private
             * 
             * @brief The random number generator used in deciding whether or not to mutate the data
             */
            std::mt19937 m_rngEngine;

            /**
            * @private
            *
            * @brief distribution for deciding whether or not to mutate the data
            */
            std::uniform_real_distribution<> m_rngDistribution;
    };

}

#endif //__ROSBAGMUTATIONPLAYER_H__
