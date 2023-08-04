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

#ifndef __GORGON_MUTATORS_FLOATS_H__
#define __GORGON_MUTATORS_FLOATS_H__

#include <limits>
#include <random>
#include <math.h>

#include "base.h"


namespace Double
{

    /**
     * @brief A Parent superclass for double-precision float Mutator types to inherit from
     *
     */

    class DoubleMutator: public BaseMutator
    {
        public:

            /**
             * @brief Accepts some data, modifies it, then returns the data
             * @note derived classes should implement this function with desired functionality
             * @note this method can't be const because implementations like stuck value requires memory
             *
             * @param data a pointer to the double value that is changed
             *
             * @returns None
             */
            virtual inline void mutate(double* data) {};

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
            virtual ~DoubleMutator() {};

    };


    /**
     * @brief Mutator that replaces the provided double with a randomly chosen value from a predefined list
     * @note the mutator's list of values contains doubles of some semantic importance (max/min, subnormals, NaN, Inf, etc)
     */
    class DictionaryAttackMutator: public DoubleMutator
    {

        public:

            /**
             * @brief creates a mutator that will replace the specified double with a random exceptional value
             */
            DictionaryAttackMutator();

            /**
             * @brief creates a mutator that will replace the specified double with a random exceptional value
             * @note seed must be provided for Constructor and mutator to guaruntee determinism
             * 
             * @param seed used to set state of rng engine
             */
            DictionaryAttackMutator(unsigned long seed);


            /**
             * @brief takes a pointer to a double and replaces it with an axceptional value
             *
             * @param data double pointer to data which will be adusted
             *
             * @return None
             */
            void mutate(double* data) override;

            /**
             * @brief takes a pointer to data and repalces it with a seeded, random exceptional value
             * @note determinism is not guarunteed unless the Mutator is constructed with a specified seed as well.
             * 
             * @param void* pointer to data
             * @param seed int used to seed any random generation
             *
             * @returns None
             */
            void mutateByReferenceSeeded(void* data, unsigned long seed) override;

            /**
             * @brief returns a random value from the private list of exceptional values
             * 
             * @returns double a random exceptional value
             */
            double getExceptionalValue();

            /**
             * @brief returns the number of values in the exceptional values array
             *
             * @returns int the number of elements in the exceptional value array
             */
            inline int getNumExceptionalValues() const
            {
                return NUM_EXCEPTIONAL_VALUES;
            }

            /**
             * @brief returns true if the given value is in the exceptional values array
             *
             * @returns bool flag if given value is in the exceptional vals array
             */
            bool valInExceptionalValuesArray(double valToCheck) const;


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
             * @brief mersenne twister engine used for getting a random exceptional value.
             */
            std::mt19937 m_rngEngine;

            /**
             * @private
             *
             * @brief disstribution generator for getting a random exceptional value
             */
            std::uniform_int_distribution<> m_rngDistribution;

            /**@private
             * @brief stored initial seed so that mutateByRefSeeded doesn't depend on order
             */
            const unsigned long m_seed;

            /**
             * @private
             *
             * @brief number which defines how many exceptional values there are in the member array
             */
            static const int NUM_EXCEPTIONAL_VALUES = 68;

            /**
             * @private
             *
             * @brief array holding values which may be useful to test (0s, 1s, max/min, powers of 2, pi/e)
             */
            const double m_exceptionalDoubleValues[NUM_EXCEPTIONAL_VALUES] = {
                // both zeros in case of division assumptions
                0.0, -0.0,
                // ones for identity multiplication/division
                1.0, -1.0,
                //powers of 2 in case values are used in integer artihmetic
                2.0, 4.0, 8.0, 16.0, 32.0, 64.0, 128.0,
                256.0, 512.0, 1024.0, 2048.0, 4096.0, 8192.0,
                16384.0, 32768.0, 65536.0,
                -2.0, -4.0, -8.0, -16.0, -32.0, -64.0, -128.0,
                -256.0, -512.0, -1024.0, -2048.0, -4096.0, -8192.0,
                -16384.0, -32768.0, -65536.0,
                // e, pi included in case values are used for unique arithmetic like yaw
                M_E, -M_E,
                M_PI, -M_PI,
                std::numeric_limits<double>::denorm_min(), -std::numeric_limits<double>::denorm_min(),  //min subnormal
                std::numeric_limits<double>::min(), -std::numeric_limits<double>::min(),    // tests low-precision
                std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest(),
                std::numeric_limits<double>::epsilon(), -std::numeric_limits<double>::epsilon(),
                // test float, int exceptionals in case these values are used elsewhere
                std::numeric_limits<float>::denorm_min(), -std::numeric_limits<float>::denorm_min(),  //min subnormal
                std::numeric_limits<float>::min(), -std::numeric_limits<float>::min(),
                std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest(),
                std::numeric_limits<float>::epsilon(), -std::numeric_limits<float>::epsilon(),
                std::numeric_limits<int>::max(), std::numeric_limits<int>::min(),
                std::numeric_limits<unsigned int>::max(), -std::numeric_limits<unsigned int>::max(),
                // classic Nan, inf injections in case code isn't prepared for extremes
                std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::signaling_NaN(),
                1.0 + std::numeric_limits<double>::epsilon(), 1.0 - std::numeric_limits<double>::epsilon(),
                -1.0 + std::numeric_limits<double>::epsilon(), -1.0 - std::numeric_limits<double>::epsilon()
            };

    };


    /**
     * @brief Mutator that adjusts a given double by a gaussian distribution
     * @note Mean and Std Deviation are provided at construction
     */
    class GaussianNoiseMutator: public DoubleMutator
    {

        public:
            /**
             * @brief creates a mutator that will adjust given doubles with the defined mean as an offset
             *
             * @param mean average offset for value
             * @param stdDev deviation of adjustments
             * 
             * @note the distribution mean defines an average offset the value will receive.
             */
            GaussianNoiseMutator(double mean, double stdDev);
      
            /**
             * @brief creates a mutator that will adjust given doubles with the defined mean as an offset
             *
             * @note the distribution mean defines an average offset the value will receive.
             * 
             * @param mean average offset for value
             * @param stdDev deviation of adjustments
             * @param seed int for seeding deterministic mutations
             */
            GaussianNoiseMutator(double mean, double stdDev, unsigned long seed);


            /**
             * @brief creates a mutator  that will adjust given doubles with no offset
             *
             * @note the distribution mean using this constructor is 0, meaning there is no average offset
             */
            GaussianNoiseMutator(double stdDev);


            /**
             * @brief takes a pointer to a double and randomly adjusts it within a gaussian distribution
             *
             * @param data double pointer to data which will be adusted
             *
             * @return None
             */
            void mutate(double* data) override;

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
             *
             * @returns double a random value within the defined distribution
             */
            double getGaussianAdjustment();

            /**
             * @brief returns the distribution's mean
             *
             * @return double representing the mean value of the distribution
             */
            inline double getMean() const
            {
                return m_gaussianMean;
            };

            /**
             * @brief returns the distribution's standard deviation
             *
             * @return double representing the standard deviation of the distribution
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
     */
    class StuckValueMutator: public DoubleMutator
    {
        public:

            /**
             * @brief creates a mutator hat always changes data to the same value
             *
             * @param stuckVal double value to change all data to
             *
             * @return none
             */
            StuckValueMutator(double stuckVal);


            /**
             * @brief change data to predetermined value
             *
             * @param data double* to data to change
             *
             * @returns none
             */
            void mutate(double* data) override;

        private:

            /**
             * @private
             * @brief value to change data to
             */
            const double m_stuckValue;

    };

}; // namespace Double

#endif