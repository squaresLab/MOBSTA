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

#include <numeric_arrays.h>

#define DECLARE_TYPES(mutatorType) \
    template class mutatorType<double>; \
    template class mutatorType<float>; \
    template class mutatorType<int>; \
    template class mutatorType<unsigned int>;

// begin FixedSizeNumericArrayMutator

template<typename T>
FixedSizeNumericArray::FixedSizeNumericArrayMutator<T>::FixedSizeNumericArrayMutator(std::size_t arraySize)
    : m_arraySize(arraySize)
{}

template<typename T>
void FixedSizeNumericArray::FixedSizeNumericArrayMutator<T>::mutateRef(void* data)
{
    T* data_ptr = static_cast<T*>(data);
    mutate(data_ptr);
}


// begin ReplaceWholeArrayMutator

template<typename T>
FixedSizeNumericArray::ReplaceWholeArrayMutator<T>::ReplaceWholeArrayMutator(T replacementValue, std::size_t arraySize)
    : FixedSizeNumericArray::FixedSizeNumericArrayMutator<T>(arraySize),
      m_replacementValue(replacementValue)
{}

template<typename T>
void FixedSizeNumericArray::ReplaceWholeArrayMutator<T>::mutate(T* data)
{
    for (unsigned int i = 0; i < this->m_arraySize; ++i) {
        data[i] = m_replacementValue;
    }
}

DECLARE_TYPES(FixedSizeNumericArray::ReplaceWholeArrayMutator)


// begin ProbabalisticReplaceMutator

template<typename T>
FixedSizeNumericArray::ProbabalisticReplaceMutator<T>::ProbabalisticReplaceMutator(T replacementValue, float probability, std::size_t arraySize)
    : FixedSizeNumericArray::ProbabalisticReplaceMutator<T>::ProbabalisticReplaceMutator(replacementValue, probability, arraySize, 0)
{}

template<typename T>
FixedSizeNumericArray::ProbabalisticReplaceMutator<T>::ProbabalisticReplaceMutator(T replacementValue, float probability, std::size_t arraySize, unsigned long seed)
    : FixedSizeNumericArray::FixedSizeNumericArrayMutator<T>(arraySize),
      m_rngDistribution(0.0, 1.0),
      m_rngEngine(std::random_device()()),
      m_seed(seed),
      m_replacementValue(replacementValue),
      m_probability(probability)
{}

template<typename T>
void FixedSizeNumericArray::ProbabalisticReplaceMutator<T>::mutate(T* data)
{
    for (unsigned int i = 0; i < this->m_arraySize; ++i) {
        if (m_rngDistribution(m_rngEngine) < m_probability) {
            data[i] = m_replacementValue;
        }
    }
}

template<typename T>
void FixedSizeNumericArray::ProbabalisticReplaceMutator<T>::mutateByReferenceSeeded(void * data, unsigned long seed)
{
    setRngEngineSeed(seed);
    this->mutateRef(data);
}

template<typename T>
void FixedSizeNumericArray::ProbabalisticReplaceMutator<T>::setRngEngineSeed(unsigned long seed)
{
    // combine both seeds to initialize the engine
    std::seed_seq seedSequence{m_seed, seed};
    m_rngEngine.seed(seedSequence);
    
    // ensures that the same result is returned for the same seed every time
    // in case a different system uses the internal state
    m_rngDistribution.reset();
}

DECLARE_TYPES(FixedSizeNumericArray::ProbabalisticReplaceMutator)


// begin AddToWholeArrayMutator

template<typename T>
FixedSizeNumericArray::AddToWholeArrayMutator<T>::AddToWholeArrayMutator(double valueToAdd, T minArrayValue, T maxArrayValue, std::size_t arraySize)
    : FixedSizeNumericArray::FixedSizeNumericArrayMutator<T>(arraySize),
      m_valueToAdd(valueToAdd),
      m_minArrayValue(minArrayValue),
      m_maxArrayValue(maxArrayValue)
{}

template<typename T>
void FixedSizeNumericArray::AddToWholeArrayMutator<T>::mutate(T* data)
{
    for (unsigned int i = 0; i < this->m_arraySize; ++i) {
        // make sure to catch overflow bugs
        double thisOffset = std::min(1.0*m_maxArrayValue - data[i], m_valueToAdd);
        thisOffset = std::max(1.0*m_minArrayValue - data[i], thisOffset);
        data[i] = static_cast<T>(data[i] + thisOffset);
    }
}

DECLARE_TYPES(FixedSizeNumericArray::AddToWholeArrayMutator)