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
#ifndef __GORGON_UTILS_H__
#define __GORGON_UTILS_H__

#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

#include <gorgon/mutators/base.h>
#include <gorgon/mutators/floats.h>
#include <gorgon/mutators/char_arrays.h>
#include <gorgon/mutators/position_2d.h>
#include <gorgon/mutators/numeric_arrays.h>
#include <gorgon/mutators/numeric.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace utils
{



    /**
     * @brief Class for creating mutator Objects from a String Type and Json Args
     * @note
     *
     */
    class MutatorFactory
    {
        public:


            MutatorFactory();


            /**
             * @brief dynamically creates a new mutator with a given seed for rng
             *
             * @param mutatorType string containing the Mutator Type name
             * @param mutatorArgs json containing the relevant construction variables needed for the mutator
             * @param seed int for seeding random generation: 
             *
             * @throws invalid_argument exception if mutatorType is not in map
             *
             * @return BaseMutator* pointer to the constructed mutator
             */
            std::shared_ptr<BaseMutator> buildMutator(std::string mutatorType, json args, unsigned long seed);

            /**
             * @brief dynamically creates a new mutator
             *
             * @param mutatorType string containing the Mutator Type name
             * @param mutatorArgs json containing the relevant construction variables needed for the mutator
             *
             * @throws invalid_argument exception if mutatorType is not in map
             *
             * @return BaseMutator* pointer to the constructed mutator
             */
            std::shared_ptr<BaseMutator> buildMutator(std::string mutatorType, json args);

        protected:
            /**
             * @private
             *
             * @brief map containing blueprints: lambda functions that create a specified mutator from an arg list json
             */
            std::map< std::string, std::function<std::shared_ptr<BaseMutator>(json, unsigned long)> > m_MutatorMap;

    };



} // namespace utils
#endif
