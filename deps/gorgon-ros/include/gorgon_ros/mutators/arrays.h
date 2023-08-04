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

#ifndef __GORGON_ROS_MUTATORS_ARRAYS_H__
#define __GORGON_ROS_MUTATORS_ARRAYS_H__

#include <cstdint>
#include <ros/serialization.h>
#include <gorgon_ros/mutators/ros_base.h>
#include "gorgon/mutators/numerical_arrays.h"

namespace ArraySerialized {

/**
 * @brief A Parent superclass for array mutator types to inherit from
 * 
 * This class is meant to be able to operate on fixed size arrays-- the constructor
 * takes a size as a parameter and then all arrays that come in are expected to be
 * that size. Calling "mutateSerialized" with an array that's shorter than the
 * expected size will cause the mutator to return without doing anything, while
 * calling "mutateRef" with an array that's shorter than the expected size will
 * cause a crash. With either function, if the incoming array is longer than
 * the expected size, then elements beyond index [expectedSize-1] will not
 * be modified
 * 
 * Current supported types for the arrays:
 * double, float, int, unsigned int
 */
template<typename T>
class NumericalArrayMutator : public RosBaseMutator
{
public:
    /**
     * @brief Constructor for the base class takes the expected array length
     *
     * @param arraySize expected length of the arrays to be modified, specified in number of elements
     * and not in number of bytes
     */
    NumericalArrayMutator(std::size_t arraySize);

    /**
     * @brief Accepts some data, modifies it, then returns the data
     *
     * @param data the array that is changed
     *
     * @returns None
     */
    virtual inline void mutate(T * data) = 0;

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
     * @brief takes a pointer to data and returns it
     * @note derived classes do not need override this function - mutate is automatically called
     *
     * @param data pointer to data
     * @param size size of the data in bytes
     *
     * @returns None
     */
    void mutateSerialized(void* data, size_t size) override;

protected:
    std::size_t m_expectedArraySize;
};



/**
 * @brief Mutator which replaces all values in the array with a predetermined value
 *
 * Current supported types for the arrays:
 * double, float, int, unsigned int
 */
template<typename T>
class ReplaceWholeArrayMutator : public NumericalArrayMutator<T>
{
public:

    /**
     * @brief creates a mutator which replaces all values in an array
     *
     * @param replacementValue new value to set the whole array to
     * @param arraySize expected length of the arrays to be modified, specified in number of elements
     * and not in number of bytes
     */
    ReplaceWholeArrayMutator(T replacementValue, std::size_t arraySize);

    /**
     * @brief Accepts some data, modifies it, then returns the data
     *
     * @param data the array that is changed
     *
     * @returns None
     */  
    void mutate(T * data) override;

private:

    /**@private
     *
     * @brief mutator that contains the implementation
     */
    FixedSizeNumericalArray::ReplaceWholeArrayMutator<T> m_replaceWholeArray;
};


/**
 * @brief Mutator which has a probability p of replacing each value in the array with a predetermined value
 *
 * Current supported types for the arrays:
 * double, float, int, unsigned int
 */
template<typename T>
class ProbabalisticReplaceMutator : public NumericalArrayMutator<T>
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
     * @param arraySize expected length of the arrays to be modified, specified in number of elements
     * and not in number of bytes
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
     * @param arraySize expected length of the arrays to be modified, specified in number of elements
     * @param seed int for seeding deterministic mutations
     * and not in number of bytes
     */
    ProbabalisticReplaceMutator(T replacementValue, float probability, std::size_t arraySize, unsigned long seed);

    /**
     * @brief Accepts some data, modifies it, then returns the data
     *
     * @param data the array that is changed
     *
     * @returns None
     */      
    void mutate(T * data) override;

    /**
     * @brief takes a pointer to data and repalces it with a seeded, random exceptional value
     * @note determinism is not guarunteed unless the Mutator is constructed with a specified seed as well.
     * 
     * @param data pointer to data
     * @param seed int used to seed any random generation
     *
     * @returns None
     */
    void mutateByReferenceSeeded(void* data, unsigned long seed) override;

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
    void mutateSerializedSeeded(void* data, size_t dataSize, unsigned long seed) override;

private:
    /**@private
     *
     * @brief mutator that contains the implementation
     */
    FixedSizeNumericalArray::ProbabalisticReplaceMutator<T> m_probabalisticReplace;
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
class AddToWholeArrayMutator : public NumericalArrayMutator<T>
{
public:
    /**
     * @brief Creates mutator which adds a constant value to all of the values in an array
     *
     * @param valueToAdd Value which gets added to each value in the array. Can be positive or negative.
     * @param minArrayValue Lower bound on what the entries of the array can be. 
     * If valueToAdd is negative, then the value at index i will be set to minmax(minArrayValue, array[i]+valueToAdd)
     * If valueToAdd is positive, then minArrayValue is ignored
     * @param maxArrayValue Upper bound on what the entries of the array can be.
     * If valueToAdd is positive, then the value at index i will be set to min(maxArrayValue, array[i]+valueToAdd)
     * If valueToAdd is negative, then maxArrayValue is ignored
     * @param arraySize expected length of the arrays to be modified, specified in number of elements
     * and not in number of bytes
     */
    AddToWholeArrayMutator(T valueToAdd, T minArrayValue, T maxArrayValue, std::size_t arraySize);

    /**
     * @brief Accepts some data, modifies it, then returns the data
     *
     * @param data the array that is changed
     *
     * @returns None
     */         
    void mutate(T* data) override;

private:

    /**@private
     *
     * @brief mutator that contains the implementation
     */
    FixedSizeNumericalArray::AddToWholeArrayMutator<T> m_addToWholeArray;
};


}; // end namespace ArraySerialized

#endif