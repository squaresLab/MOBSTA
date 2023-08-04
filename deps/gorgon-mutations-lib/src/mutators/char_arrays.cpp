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


#include <char_arrays.h>

// begin ChararrayMutator
void CharArray::CharArrayMutator::mutateRef(void* data, std::size_t size)
{
    char* data_ptr = static_cast<char*>(data);
    mutate(data_ptr, size);
}

// begin ReplaceCharMutator


CharArray::ReplaceCharMutator::ReplaceCharMutator(char replacement, int index)
    : m_replacementChar(replacement)
    , m_replacementIndex(index)
{ }


void CharArray::ReplaceCharMutator::mutate(char* data, std::size_t size)
{

    if(m_replacementIndex < size)
    {
        data[m_replacementIndex] = m_replacementChar;
    }


}