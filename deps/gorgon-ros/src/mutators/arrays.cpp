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


#include <gorgon_ros/mutators/arrays.h>
#include <ros/console.h>

#define DECLARE_TYPES(mutatorType) \
    template class mutatorType<double>; \
    template class mutatorType<float>; \
    template class mutatorType<int>; \
    template class mutatorType<unsigned int>;


// begin NumericalArrayMutator

template<typename T>
ArraySerialized::NumericalArrayMutator<T>::NumericalArrayMutator(std::size_t arraySize)
    : m_expectedArraySize(arraySize)
{}

template<typename T>
void ArraySerialized::NumericalArrayMutator<T>::mutateRef(void * data)
{
    T* data_ptr = static_cast<T*>(data);
    mutate(data_ptr);
}

template<typename T>
void ArraySerialized::NumericalArrayMutator<T>::mutateSerialized(void* data, size_t dataSize)
{
    if (dataSize/sizeof(T) >= m_expectedArraySize)
    {
        T* data_ptr = static_cast<T*>(data);
        mutate(data_ptr);
    }
    else
    {
        ROS_WARN("Warning: skipping array message because length of %lu is shorter than expected length %lu", dataSize/sizeof(T), m_expectedArraySize);
    }
}


// begin ReplaceWholeArrayMutator

template<typename T>
ArraySerialized::ReplaceWholeArrayMutator<T>::ReplaceWholeArrayMutator(T replacementValue, std::size_t arraySize)
    : ArraySerialized::NumericalArrayMutator<T>(arraySize),
      m_replaceWholeArray(replacementValue, arraySize)
{}

template<typename T>
void ArraySerialized::ReplaceWholeArrayMutator<T>::mutate(T* data)
{
    m_replaceWholeArray.mutate(data);
}

DECLARE_TYPES(ArraySerialized::ReplaceWholeArrayMutator)


// begin ProbabalisticReplaceMutator

template<typename T>
ArraySerialized::ProbabalisticReplaceMutator<T>::ProbabalisticReplaceMutator(T replacementValue, float probability, std::size_t arraySize)
    : ArraySerialized::NumericalArrayMutator<T>(arraySize),
      m_probabalisticReplace(replacementValue, probability, arraySize)
{}

template<typename T>
ArraySerialized::ProbabalisticReplaceMutator<T>::ProbabalisticReplaceMutator(T replacementValue, float probability, std::size_t arraySize, unsigned long seed)
    : ArraySerialized::NumericalArrayMutator<T>(arraySize),
      m_probabalisticReplace(replacementValue, probability, arraySize, seed)
{}

template<typename T>
void ArraySerialized::ProbabalisticReplaceMutator<T>::mutate(T* data)
{
    m_probabalisticReplace.mutate(data);
}

template<typename T>
void ArraySerialized::ProbabalisticReplaceMutator<T>::mutateByReferenceSeeded(void* data, unsigned long seed)
{
    m_probabalisticReplace.mutateByReferenceSeeded(data, seed);
}

template<typename T>
void ArraySerialized::ProbabalisticReplaceMutator<T>::mutateSerializedSeeded(void* data, std::size_t size, unsigned long seed)
{
    if (size/sizeof(T) >= this->m_expectedArraySize)
    {
        m_probabalisticReplace.mutateByReferenceSeeded(data, seed);
    }
    else
    {
        ROS_WARN("Warning: skipping array message because length of %lu is shorter than expected length %lu", size/sizeof(T), this->m_expectedArraySize);
    }
}

DECLARE_TYPES(ArraySerialized::ProbabalisticReplaceMutator)


// begin AddToWholeArrayMutator

template<typename T>
ArraySerialized::AddToWholeArrayMutator<T>::AddToWholeArrayMutator(T valueToAdd, T minArrayValue, T maxArrayValue, std::size_t arraySize)
    : ArraySerialized::NumericalArrayMutator<T>(arraySize),
      m_addToWholeArray(valueToAdd, minArrayValue, maxArrayValue, arraySize)
{}

template<typename T>
void ArraySerialized::AddToWholeArrayMutator<T>::mutate(T* data)
{
    m_addToWholeArray.mutate(data);
}

DECLARE_TYPES(ArraySerialized::AddToWholeArrayMutator)