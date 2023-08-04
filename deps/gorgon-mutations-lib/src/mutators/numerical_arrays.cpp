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

#include <numerical_arrays.h>

#define DECLARE_TYPES(mutatorType) \
    template class mutatorType<double>; \
    template class mutatorType<float>; \
    template class mutatorType<int>; \
    template class mutatorType<unsigned int>;

// begin FixedSizeNumericalArrayMutator

template<typename T>
FixedSizeNumericalArray::FixedSizeNumericalArrayMutator<T>::FixedSizeNumericalArrayMutator(std::size_t arraySize)
    : m_arraySize(arraySize)
{}

template<typename T>
void FixedSizeNumericalArray::FixedSizeNumericalArrayMutator<T>::mutateRef(void* data)
{
    T* data_ptr = static_cast<T*>(data);
    mutate(data_ptr);
}


// begin ReplaceWholeArrayMutator

template<typename T>
FixedSizeNumericalArray::ReplaceWholeArrayMutator<T>::ReplaceWholeArrayMutator(T replacementValue, std::size_t arraySize)
    : FixedSizeNumericalArray::FixedSizeNumericalArrayMutator<T>(arraySize),
      m_replacementValue(replacementValue)
{}

template<typename T>
void FixedSizeNumericalArray::ReplaceWholeArrayMutator<T>::mutate(T* data)
{
    for (unsigned int i = 0; i < this->m_arraySize; ++i) {
        data[i] = m_replacementValue;
    }
}

DECLARE_TYPES(FixedSizeNumericalArray::ReplaceWholeArrayMutator)


// begin ProbabalisticReplaceMutator

template<typename T>
FixedSizeNumericalArray::ProbabalisticReplaceMutator<T>::ProbabalisticReplaceMutator(T replacementValue, float probability, std::size_t arraySize)
    : FixedSizeNumericalArray::ProbabalisticReplaceMutator<T>::ProbabalisticReplaceMutator(replacementValue, probability, arraySize, 0)
{}

template<typename T>
FixedSizeNumericalArray::ProbabalisticReplaceMutator<T>::ProbabalisticReplaceMutator(T replacementValue, float probability, std::size_t arraySize, unsigned long seed)
    : FixedSizeNumericalArray::FixedSizeNumericalArrayMutator<T>(arraySize),
      m_rngDistribution(0.0, 1.0),
      m_rngEngine(std::random_device()()),
      m_seed(seed),
      m_replacementValue(replacementValue),
      m_probability(probability)
{}

template<typename T>
void FixedSizeNumericalArray::ProbabalisticReplaceMutator<T>::mutate(T* data)
{
    for (unsigned int i = 0; i < this->m_arraySize; ++i) {
        if (m_rngDistribution(m_rngEngine) < m_probability) {
            data[i] = m_replacementValue;
        }
    }
}

template<typename T>
void FixedSizeNumericalArray::ProbabalisticReplaceMutator<T>::mutateByReferenceSeeded(void * data, unsigned long seed)
{
    setRngEngineSeed(seed);
    this->mutateRef(data);
}

template<typename T>
void FixedSizeNumericalArray::ProbabalisticReplaceMutator<T>::setRngEngineSeed(unsigned long seed)
{
    // combine both seeds to initialize the engine
    std::seed_seq seedSequence{m_seed, seed};
    m_rngEngine.seed(seedSequence);
    
    // ensures that the same result is returned for the same seed every time
    // in case a different system uses the internal state
    m_rngDistribution.reset();
}

DECLARE_TYPES(FixedSizeNumericalArray::ProbabalisticReplaceMutator)


// begin AddToWholeArrayMutator

template<typename T>
FixedSizeNumericalArray::AddToWholeArrayMutator<T>::AddToWholeArrayMutator(T valueToAdd, T minArrayValue, T maxArrayValue, std::size_t arraySize)
    : FixedSizeNumericalArray::FixedSizeNumericalArrayMutator<T>(arraySize),
      m_valueToAdd(valueToAdd),
      m_minArrayValue(minArrayValue),
      m_maxArrayValue(maxArrayValue)
{}

template<typename T>
void FixedSizeNumericalArray::AddToWholeArrayMutator<T>::mutate(T* data)
{
    for (unsigned int i = 0; i < this->m_arraySize; ++i) {
        data[i] = std::max(m_minArrayValue, data[i]+m_valueToAdd);
        data[i] = std::min(data[i], m_maxArrayValue);
    }
}

DECLARE_TYPES(FixedSizeNumericalArray::AddToWholeArrayMutator)