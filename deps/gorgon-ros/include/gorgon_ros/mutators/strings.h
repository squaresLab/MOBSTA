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

#ifndef __GORGON_ROS_MUTATORS_STRINGS_H__
#define __GORGON_ROS_MUTATORS_STRINGS_H__

#include <cstdint>
#include <ros/serialization.h>
#include "std_msgs/String.h"
#include <gorgon_ros/mutators/ros_base.h>
#include "gorgon/mutators/char_arrays.h"




namespace StringSerialized
{


    /**
     * @brief A Parent superclass for string Mutator types to inherit from
     * @note String sizes cannot be changed
     *
     */
    class StringMutator: public RosBaseMutator
    {
        public:

            /**
             * @brief Accepts some data, modifies it, then returns the data
             * @note derived classes should implement this function with desired functionality
             * @note this method can't be const because implementations like stuck value requires memory
             * @note String size cannot be changed
             *
             * @param data a pointer to the string that is changed
             *
             * @returns None
             */
            virtual inline void mutate(std_msgs::String* data) =0;


            /**
             * @brief takes a pointer to data and returns it
             * @note derived classes do not need override this function - mutate is automatically called
             *
             * @param data pointer to data
             *
             * @returns None
             */
            void mutateRef(void* data) override;

            /**
             * @brief accepts
             * @param dataSize length of data in bytes
             *
             */
            void mutateSerialized(void* data, size_t dataSize) override;
    };


    /**
     * @brief Mutator which replaces a character at index i in a string with a predetermined char
     *
     */
    class ReplaceCharMutator: public StringMutator
    {
        public:

            /**
             * @brief creates mutator which replaces a char in a string
             *
             * @param replacement char to put in place at index
             * @param index int where to put replacement char in string
             */
            ReplaceCharMutator(char replacment, int index);

            /**
             * @brief takes pointer to serialized string, interprets length, checks if mutatable, then replaces char at index
             * @note will not replace anything if string not long enough
             *
             * @param data char* to serialized string data
             */
            void mutate(std_msgs::String* data);


        private:

            /**@private
             * @brief replaceCharacter mutator that this implements
             */
            CharArray::ReplaceCharMutator m_replaceChar;


    };


}; // end namespace StringSerialized
#endif