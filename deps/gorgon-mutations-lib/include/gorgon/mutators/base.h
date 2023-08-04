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


#ifndef __GORGON_MUTATORS_BASE_H__
#define __GORGON_MUTATORS_BASE_H__


/**
 * @brief A Parent superclass for Mutator types to inherit from. Be default changes nothing
 * @note exists as a blank mutator and for inhereitence by additional Mutator Types
 *
 */
class BaseMutator
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
        virtual inline void mutateRef(void* data) {};

        /**
         * @brief takes a pointer to data and mutates with the given seed, if applicable
         * @note Non-random derived classes should not override this function.
         * Non-random Mutators will discard the seed
         * @note Randomized Mutators should override this method with a seed-based 
         * deterministic mutation. Determinism should combine seeds provided in the 
         * Constructor and the Method to follow the determinism model.
         * 
         * @param void* pointer to data
         * @param seed int used to seed any random generation
         *
         * @returns None
         */
        virtual inline void mutateByReferenceSeeded(void* data, unsigned long seed) { mutateRef(data);}


};

#endif