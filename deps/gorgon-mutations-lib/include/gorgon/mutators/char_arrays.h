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


#ifndef __GORGON_MUTATORS_CHAR_ARRAY_H__
#define __GORGON_MUTATORS_CHAR_ARRAY_H__

#include <cstdint>

#include "base.h"


namespace CharArray {


/**
 * @brief A Parent superclass for string Mutator types to inherit from
 *
 */
class CharArrayMutator: public BaseMutator
{
public:

    /**
     * @brief Accepts some data, modifies it, then returns the data
     * @note derived classes should implement this function with desired functionality
     * @note this method can't be const because implementations like stuck value requires memory
     * @note size cannot be changed
     *
     * @param data a pointer to the char array that is changed
     * @param size how big the char array is
     *
     * @returns None
     */
    virtual inline void mutate(char* data, std::size_t size) = 0;

    /**
     * @brief takes a pointer to data and returns it
     * @note derived classes do not need override this function - mutate is automatically called
     *
     * @param void* pointer to data
     * @param size sie of char array in bytes
     *
     * @returns None
     */
    void mutateRef(void* data, std::size_t size);

};


/**
 * @brief Mutator which replaces a character at index i in a char array with a predetermined char
 *
 */
class ReplaceCharMutator: public CharArrayMutator
{
public:

    /**
     * @brief creates mutator which replaces a char in a char array
     *
     * @param replacement char to put in place at index
     * @param index int where to put replacement char in char array
     */
    ReplaceCharMutator(char replacment, int index);

    /**
     * @brief takes pointer to char array , interprets length, checks if mutatable, then replaces char at index
     * @note will not replace anything if string not long enough
     *
     * @param data char* to char array
     * @param size size of char array
     */
    void mutate(char* data, std::size_t size);


private:

    /**@private
     *
     * @brief character to replace at defined index
     */
    const char m_replacementChar;

    /**@private
     *
     * @brief where to replace defined character
     */
    const int m_replacementIndex;


};


}; // end namespace CharArray
#endif