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

#include <random>
#include <numeric.h>

#define DECLARE_TYPES(mutatorType) \
    template class mutatorType<double>; \
    template class mutatorType<float>; \
    template class mutatorType<int>; \
    template class mutatorType<unsigned int>;


// Begin NumericMutator
template<typename T>
void Numeric::NumericMutator<T>::mutateRef(void* data)
{
    T* data_ptr = static_cast<T*>(data);
    mutate(data_ptr);
}




// Begin GaussianNoise
template<typename T>
Numeric::GaussianNoiseMutator<T>::GaussianNoiseMutator(double mean, double stdDev)
    : m_rngDistribution(mean, stdDev)
    , m_rngEngine(std::random_device()())
    , m_gaussianMean(mean)
    , m_gaussianStdDev(stdDev)
    , m_seed(0)
{ }

template<typename T>
Numeric::GaussianNoiseMutator<T>::GaussianNoiseMutator(double mean, double stdDev, unsigned long seed)
    : m_rngDistribution(mean, stdDev)
    , m_rngEngine(seed)
    , m_gaussianMean(mean)
    , m_gaussianStdDev(stdDev)
    , m_seed(seed)
{ }

// delegative constructor using default mean of 0.0
template<typename T>
Numeric::GaussianNoiseMutator<T>::GaussianNoiseMutator(double stdDev)
    : GaussianNoiseMutator<T>::GaussianNoiseMutator(0.0, stdDev)
{ }

template<typename T>
void Numeric::GaussianNoiseMutator<T>::mutate(T* data)
{
    *data += getGaussianAdjustment();
}

template<typename T>
void Numeric::GaussianNoiseMutator<T>::mutateByReferenceSeeded(void* data, unsigned long seed)
{
    setRngEngineSeed(seed);
    this->mutateRef(data);
}

template<typename T>
T Numeric::GaussianNoiseMutator<T>::getGaussianAdjustment()
{
    T adjustment = static_cast<T>(m_rngDistribution(m_rngEngine));
    return adjustment;
}

template<typename T>
void Numeric::GaussianNoiseMutator<T>::setRngEngineSeed(unsigned long seed)
{
    // combine both seeds to initialize the engine
    std::seed_seq seedSequence{m_seed, seed};
    m_rngEngine.seed(seedSequence);

    // ensures that the same result is returned for the same seed every time
    // as each distribution call is dependant on previous call unless reset
    m_rngDistribution.reset();
}
DECLARE_TYPES(Numeric::GaussianNoiseMutator)
// End GaussianNoise


// begin StuckValue
template<typename T>
Numeric::StuckValueMutator<T>::StuckValueMutator(T stuckVal)
    : m_stuckValue(stuckVal)
{ }

template<typename T>
void Numeric::StuckValueMutator<T>::mutate(T* data)
{
    *data = m_stuckValue;
}
DECLARE_TYPES(Numeric::StuckValueMutator)
// End StuckValue
