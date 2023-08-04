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

#ifndef __GORGON_MUTATORS_NUMERIC_H__
#define __GORGON_MUTATORS_NUMERIC_H__

#include <limits>
#include <random>
#include <math.h>

#include "base.h"


namespace Numeric
{

    /**
     * @brief A Parent superclass for single number Mutator types to inherit from
     * @tparam numeric type (float/double/uint/int)
     */
    template<typename T>
    class NumericMutator: public BaseMutator
    {
        public:

            /**
             * @brief Accepts some data, modifies it, then returns the data
             * @note derived classes should implement this function with desired functionality
             * @note this method can't be const because implementations like stuck value requires memory
             *
             * @param data a pointer to the number that is changed
             *
             * @returns None
             */
            virtual inline void mutate(T* data) = 0;

            /**
             * @brief takes a pointer to data and returns it
             * @note derived classes do not need override this function - mutate is automatically called
             *
             * @param void* pointer to data
             *
             * @returns None
             */
            void mutateRef(void* data) override;

            /**
             * @brief make class pure-virtual by virtualizing destructor
             */
            virtual ~NumericMutator() {};

    };


    /**
     * @brief Mutator that adjusts a given number by a gaussian distribution
     * @note Mean and Std Deviation are provided at construction
     * @tparam numeric type (float/double/uint/int)
     */
    template<typename T>
    class GaussianNoiseMutator: public NumericMutator<T>
    {

        public:
            /**
             * @brief creates a mutator that will adjust given number with the defined mean as an offset
             *
             * @param mean average offset for value
             * @param stdDev deviation of adjustments
             * 
             * @note the distribution mean defines an average offset the value will receive.
             */
            GaussianNoiseMutator(double mean, double stdDev);
      
            /**
             * @brief creates a mutator that will adjust given number with the defined mean as an offset
             *
             * @note the distribution mean defines an average offset the value will receive.
             * 
             * @param mean average offset for value
             * @param stdDev deviation of adjustments
             * @param seed int for seeding deterministic mutations
             */
            GaussianNoiseMutator(double mean, double stdDev, unsigned long seed);


            /**
             * @brief creates a mutator that will adjust given number with no offset
             *
             * @note the distribution mean using this constructor is 0, meaning there is no average offset
             */
            GaussianNoiseMutator(double stdDev);


            /**
             * @brief takes a pointer to a number and randomly adjusts it within a gaussian distribution
             *
             * @param data pointer to data which will be adusted
             *
             * @return None
             */
            void mutate(T* data) override;

            /**
             * @brief takes a pointer to data and modifies it with a seeded gaussian adjustment
             * @note determinism is not guarunteed unless the Mutator is constructed with a specified seed as well.
             * 
             * @param void* pointer to data
             * @param seed int used to seed any random generation
             *
             * @returns None
             */
            void mutateByReferenceSeeded(void* data, unsigned long seed) override;

            /**
             * @brief returns a random value within the constructed gaussian distribution
             * @note casts double value to the template parameter
             * 
             * @returns number a random value within the defined distribution
             */
            T getGaussianAdjustment();

            /**
             * @brief returns the distribution's mean
             *
             * @return number representing the mean value of the distribution
             */
            inline double getMean() const
            {
                return m_gaussianMean;
            };

            /**
             * @brief returns the distribution's standard deviation
             *
             * @return number representing the standard deviation of the distribution
             */
            inline double getStdDev() const
            {
                return m_gaussianStdDev;
            };

        private:

            /**@private
             * @brief seeds the internal engine with the given seed and the constructed seed
             * 
             * @param seed int used as secondary seed for the random engine
             * 
             * @returns None
             */
            void setRngEngineSeed(unsigned long seed);

            /**
             * @private
             *
             * @brief mersenne twister engine used for getting a random gaussian adjustment
             */
            std::mt19937 m_rngEngine;

            /**
             * @private
             *
             * @brief disstribution generator for adjusting the given float randomly
             */
            std::normal_distribution<> m_rngDistribution;

            /**@private
             * @brief stored initial seed so that mutateByRefSeeded doesn't depend on order
             */
            const unsigned long m_seed;

            /**
             * @private
             *
             * @brief number which defines what mean to use in the gaussian distribution
             */
            double m_gaussianMean;

            /**
             * @private
             *
             * @brief number which defines what standard deviation to use in the gaussian distribution
             */
            double m_gaussianStdDev;
    };

    /**
     * @brief mutator that always sets a value to the same thing
     * @tparam numeric type (float/double/uint/int)
     */
    template<typename T>
    class StuckValueMutator: public NumericMutator<T>
    {
        public:

            /**
             * @brief creates a mutator that always changes data to the same value
             *
             * @param stuckVal number to change all data to
             *
             * @return none
             */
            StuckValueMutator(T stuckVal);


            /**
             * @brief change data to predetermined value
             *
             * @param data T* to data to change
             *
             * @returns none
             */
            void mutate(T* data) override;

        private:

            /**
             * @private
             * @brief value to change data to
             */
            const T m_stuckValue;

    };

}; // namespace Double

#endif