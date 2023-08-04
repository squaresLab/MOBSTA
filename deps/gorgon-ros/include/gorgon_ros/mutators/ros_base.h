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


#ifndef __GORGON_ROS_MUTATORS_BASE_H__
#define __GORGON_ROS_MUTATORS_BASE_H__


/**
 * @brief A Parent superclass for Mutator types to inherit from. The default changes nothing
 * @note exists as a blank mutator and for inhereitence by additional Mutator Types
 *
 */
class RosBaseMutator
{
    public:
        /**
         * @brief takes a pointer to data and returns it
         * @note derived classes should override this function with specific ways of mutating data
         *
         * @param void* pointer to data
         *
         * @returns None
         */
        virtual inline void mutateRef(void* data) = 0;

        /**
         * @brief takes a pointer data and mutates with a seed, if applicable
         * @note determinism is not guarunteed unless the Mutator is constructed with a specified seed as well.
         * 
         * @param data pointer to data
         * @param seed int used to seed any random generation
         *
         * @returns None
         */
        virtual inline void mutateByReferenceSeeded(void* data, unsigned long seed) { mutateRef(data);}

        /**
         * @brief takes serialized data and size, deserializes into a message, then reserializes into place
         *
         * @param data pointer to expected data
         * @param dataSize size in bytes of serialized data
         *
         */
        virtual inline void mutateSerialized(void* data, size_t dataSize) = 0;

        /**
         * @brief takes a pointer to serialized data and mutates with the given seed, if applicable
         * @note derived classes should override this function with specific ways of mutating data
         * @note Mutators which don't change result for each input will discard the seed
         * @note does not guaruntee same amount of randomness using a default seed
         * 
         * @param data pointer to serialized data
         * @param dataSize size of serialized data in bytes
         * @param seed int used to seed any random generation
         *
         * @returns None
         */
        virtual inline void mutateSerializedSeeded(void* data, size_t dataSize, unsigned long seed) 
            { mutateSerialized(data, dataSize);}


};

#endif