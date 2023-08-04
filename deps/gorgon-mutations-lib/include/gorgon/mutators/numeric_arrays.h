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

#ifndef __GORGON_MUTATORS_NUMERIC_ARRAYS_H__
#define __GORGON_MUTATORS_NUMERIC_ARRAYS_H__

#include <random>
#include <cstdint>
#include "base.h"

namespace FixedSizeNumericArray {

/**
 * @brief Parent class for mutators that operate on fixed size arrays of numbers
 * 
 * Here, "fixed size" means that the size of the arrays is specified in the
 * constructor. Size should be specified in terms of the number of elements in
 * the array, not the size in bytes.
 * 
 * Current supported types for the arrays:
 * double, float, int, unsigned int
 */
template<typename T>
class FixedSizeNumericArrayMutator : public BaseMutator
{
public:
    /**
     * @brief Constructor takes the expected size of the arrays that will be mutated,
     * given as the number of elements in the array, not the number of bytes
     * 
     * @param arraySize expected length of the arrays to be modified
     */ 
    FixedSizeNumericArrayMutator(std::size_t arraySize);

    /**
     * @brief Function that modifies the data. Subclasses should override this, treating
     * "data" as an array with a length given by the "size" parameter to the constructor.
     * 
     * @param data the array to be modified
     */ 
    virtual inline void mutate(T * data) = 0;

    /**
     * @brief takes a pointer to data and returns it
     * @note derived classes do not need override this function - mutate is automatically called
     *
     * @param void* pointer to data
     *
     * @returns None
     */
    void mutateRef(void* data) override;

protected:
    std::size_t m_arraySize;
};

/**
 * @brief Mutator which replaces all values in the array with a predetermined value
 * 
 * Current supported types for the arrays:
 * double, float, int, unsigned int
 */
template<typename T>
class ReplaceWholeArrayMutator : public FixedSizeNumericArrayMutator<T>
{
public:

    /**
     * @brief creates a mutator which replaces all values in an array
     *
     * @param replacementValue new value to set the whole array to
     * @param arraySize expected length of the arrays to be modified
     */
    ReplaceWholeArrayMutator(T replacementValue, std::size_t arraySize);

    /**
     * @brief Function that performs the mutation
     *
     * @param data the array that is changed
     *
     * @returns None
     */  
    void mutate(T * data) override;

private:

    /**@private
     *
     * @brief value which replaces all the values in the array
     */
    const T m_replacementValue;
};


/**
 * @brief Mutator which has a probability p of replacing each value in the array with a predetermined value
 * 
 * Current supported types for the arrays:
 * double, float, int, unsigned int
 */
template<typename T>
class ProbabalisticReplaceMutator : public FixedSizeNumericArrayMutator<T>
{
public:

    /**
     * @brief creates a mutator which has a probability p of replacing each value
     * in the array with a predetermined value. The effect of this mutator is equivalent
     * to something like:
     *
     * for each index i in array:
     *     if rand() < probability:
     *         array[i] = replacement value
     *
     * @param replacementValue new value to set array entries to
     * @param probability the probability that each entry in the array gets replaced, specified in the range [0, 1]
     * @param arraySize expected length of the arrays to be modified
     */
    ProbabalisticReplaceMutator(T replacementValue, float probability, std::size_t arraySize);

    /**
     * @brief creates a mutator which has a probability p of replacing each value
     * in the array with a predetermined value. The effect of this mutator is equivalent
     * to something like:
     *
     * for each index i in array:
     *     if rand() < probability:
     *         array[i] = replacement value
     *
     * @param replacementValue new value to set array entries to
     * @param probability the probability that each entry in the array gets replaced, specified in the range [0, 1]
     * @param arraySize expected length of the arrays to be modified
     * @param seed int for seeding deterministic mutations
     */
    ProbabalisticReplaceMutator(T replacementValue, float probability, std::size_t arraySize, unsigned long seed);

    /**
     * @brief Function that performs the mutation
     *
     * @param data the array that is changed
     *
     * @returns None
     */       
    void mutate(T * data) override;

    /**
     * @brief Function that performs the mutation after seeding the random number generator
     * @note determinism is not guarunteed unless the Mutator is constructed with a specified seed as well.
     * 
     * @param data pointer to data
     * @param seed int used to seed any random generation
     *
     * @returns None
     */
    void mutateByReferenceSeeded(void* data, unsigned long seed) override;

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
    std::uniform_real_distribution<float> m_rngDistribution;

    /**@private
        * @brief stored initial seed so that mutateByRefSeeded doesn't depend on order
        */
    const unsigned long m_seed;

    /**@private
     *
     * @brief value which might replace each value in the array
     */
    const T m_replacementValue;

    /**@private
     *
     * @brief probability that each value gets replaced
     */
    const float m_probability;
};



/**
 * @brief Mutator which adds a constant value to all of the values in an array.
 * This constant value can be positive OR negative, so basically you can use this mutator to
 * add or subtract values. You can also optionally specify min and/or max values to "cap" the
 * values in the array.
 * 
 * Current supported types for the arrays:
 * double, float, int, unsigned int
 */
template<typename T>
class AddToWholeArrayMutator : public FixedSizeNumericArrayMutator<T>
{
public:
    /**
     * @brief Creates mutator which adds a constant value to all of the values in an array
     *
     * @note Value to Add is a double for now to prevent unsigned overflow. See GOR-136
     * 
     * @param valueToAdd Value which gets added to each value in the array. Can be positive or negative.
     * @param minArrayValue Lower bound on what the entries of the array can be. 
     * If valueToAdd is negative, then the value at index i will be set to max(minArrayValue, array[i]+valueToAdd)
     * If valueToAdd is positive, then minArrayValue is ignored
     * @param maxArrayValue Upper bound on what the entries of the array can be.
     * If valueToAdd is positive, then the value at index i will be set to min(maxArrayValue, array[i]+valueToAdd)
     * If valueToAdd is negative, then maxArrayValue is ignored
     * @param arraySize expected length of the arrays to be modified
     */
    AddToWholeArrayMutator(double valueToAdd, T minArrayValue, T maxArrayValue, std::size_t arraySize);

    /**
     * @brief Function that performs the mutation
     *
     * @param data the array that is changed
     *
     * @returns None
     */              
    void mutate(T* data);

private:

    /**@private
     *
     * @brief Value which gets added to each value in the array
     */
    const double m_valueToAdd;

    /**@private
     *
     * @brief Lower bound on what the entries of the array can be
     */
    const T m_minArrayValue;

    /**@private
     *
     * @brief Upper bound on what the entries of the array can be
     */
    const T m_maxArrayValue;    
};


}; // end namespace FixedSizeNumericArray

#endif