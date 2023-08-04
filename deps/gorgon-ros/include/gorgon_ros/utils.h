/****
* © 2022 Carnegie Mellon University. All rights reserved.
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
#ifndef __GORGON_ROS_UTILS_H__
#define __GORGON_ROS_UTILS_H__

#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

#include <gorgon/mutators/base.h>
#include <gorgon/utils.h>

#include <gorgon_ros/mutators/ros_base.h>
#include <gorgon_ros/mutators/point_msgs.h>
#include <gorgon_ros/mutators/strings.h>
#include <gorgon_ros/mutators/arrays.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace utils
{

    /**
     * @brief Class that wraps gorgon mutators with the same api as the ros ones
     *
     */
    class RosWrapperMutator: public RosBaseMutator
    {
        public:
            /**
             * @brief Wrapper for non-ros gorgon mutators
             *
             * @param wrappedMutator GorgonMutationsLibrary Mutator to wrap for the GorgonRos api
             */
            RosWrapperMutator(std::shared_ptr<BaseMutator> wrappedMutator);

            /**
             * @brief Takes a pointer to data and passes it on the specified Gorgon Mutator
             *
             * @param data pointer to the data to be mutated, must match expected mutator type
             * @param data_size size in bytes of the data, usually ignored by
             *      gorgon mutators which already know size
             */
            void mutateSerialized(void* data, size_t data_size);

            /**
             * @brief takes a pointer to data of aspecified type and mutates it in place
             *
             * @param data pointer to expected data
             */
            void mutateRef(void* data);

            /**
             * @brief takes a pointer to data and  with the given seed
             * @note Mutators which don't change result for each input will discard the seed
             * @note does not guaruntee same amount of randomness using a default seed
             * 
             * @param void* pointer to data
             * @param seed int used to seed any random generation
             *
             * @returns None
             */
            void mutateByReferenceSeeded(void* data, unsigned long seed);

            /**
             * @brief takes a pointer to serialized data and passes it on to the wrapped mutator with the given seed\
             * @note Mutators which don't change result for each input will discard the seed
             * @note does not guaruntee same amount of randomness using a default seed
             * 
             * @param data pointer to serialized data
             * @param dataSize size of serialized data in bytes
             * @param seed int used to seed any random generation
             *
             * @returns None
             */
            void mutateSerializedSeeded(void* data, size_t dataSize, unsigned long seed);



        private:

            /**@private
             * @brief Pointer to Gorgon Mutator this class wraps
             */
            std::shared_ptr<BaseMutator> m_wrappedMutator;
    };


    /**
     * @brief Class for creating Gorgon Ros mutators and Gorgon Mutators at runtime
     */
    class RosMutatorFactory
    {

        public:

            /**
             * @brief constructs RosMutatorFactory
             */
            RosMutatorFactory();

            /**
             * @brief dynamically creates a new mutator with a given seed for rng
             *
             * @param mutatorType string containing the Mutator Type name
             * @param mutatorArgs json containing the relevant construction variables needed for the mutator
             * @param seed int for seeding random generation: 
             *
             * @throws invalid_argument exception if mutatorType is not in map
             *
             * @return RosBaseMutator* pointer to the constructed mutator
             */
            std::shared_ptr<RosBaseMutator> buildMutator(std::string mutatorType, json args, unsigned long seed);


            /**
             * @brief dynamically creates a new mutator
             *
             * @param mutatorType string containing the Mutator Type name
             * @param mutatorArgs json containing the relevant construction variables needed for the mutator
             *
             * @throws invalid_argument exception if mutatorType is not in map
             *
             * @return RosBaseMutator* pointer to the constructed mutator
             */
            std::shared_ptr<RosBaseMutator> buildMutator(std::string mutatorType, json args);

        private:
            /**@private
             * @brief Mutator factory used to get base gorgon mutators wrapped in a RosWrapperMutator
             */
            MutatorFactory m_gorgonDefaultFactory;

            /**
             * @private
             *
             * @brief map containing blueprints: lambda functions that create a specified mutator from an arg list json
             */
            std::map< std::string, std::function<std::shared_ptr<RosBaseMutator>(json, unsigned long)> > m_MutatorMap;

    };


}


#endif