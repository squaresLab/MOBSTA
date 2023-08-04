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

#include <random>
#include <floats.h>
#include <iostream>

// Begin DoubleMutator

void Double::DoubleMutator::mutateRef(void* data)
{
    double* data_ptr = static_cast<double*>(data);
    mutate(data_ptr);
}



// Begin DictionaryAttack
Double::DictionaryAttackMutator::DictionaryAttackMutator()
    : m_rngDistribution(0, NUM_EXCEPTIONAL_VALUES-1)
    , m_rngEngine(std::random_device()())
    , m_seed(0)
{ }

Double::DictionaryAttackMutator::DictionaryAttackMutator(unsigned long seed)
    : m_rngDistribution(0, NUM_EXCEPTIONAL_VALUES-1)
    , m_rngEngine(seed) // used for non-seeded mutates
    , m_seed(seed)
{ }

void Double::DictionaryAttackMutator::mutate(double* data)
{
    *data = getExceptionalValue();
}

void Double::DictionaryAttackMutator::mutateByReferenceSeeded(void* data, unsigned long seed)
{
    setRngEngineSeed(seed);
    mutateRef(data);
}

double Double::DictionaryAttackMutator::getExceptionalValue()
{
    int index = m_rngDistribution(m_rngEngine);
    return m_exceptionalDoubleValues[index];
}

bool Double::DictionaryAttackMutator::valInExceptionalValuesArray(double valToCheck) const
{
    bool resultInExceptionals = false;

     // need to do special check for NaN cause any comparison w/ nan is always false

    if (std::isnan(valToCheck))
    {
        resultInExceptionals = true;
    }

    else
    {
        for (int i = 0; i < NUM_EXCEPTIONAL_VALUES; i++) {
            if (valToCheck == m_exceptionalDoubleValues[i]) {
                resultInExceptionals = true;
                break;
            }
        }
    }

    return resultInExceptionals;
}


void Double::DictionaryAttackMutator::setRngEngineSeed(unsigned long seed)
{
    // combine both seeds to initialize the engine
    std::seed_seq seedSequence{m_seed, seed};
    m_rngEngine.seed(seedSequence);
    
    // ensures that the same result is returned for the same seed every time
    // in case a different system uses the internal state
    m_rngDistribution.reset();
}
// End DictionaryAttack


//expose class to C for python wrapper
extern "C" {

    //Dict Attack
    Double::DictionaryAttackMutator* doubles_DictionaryAttackMutator_new()
    {
        return new Double::DictionaryAttackMutator();
    }
    void doubles_DictionaryAttackMutator_mutate(Double::DictionaryAttackMutator* dictAttack, double* d)
    {
        dictAttack->mutate(d);
    }
    double doubles_DictionaryAttackMutator_getExceptionalValue(Double::DictionaryAttackMutator* dictAttack)
    {
        return dictAttack->getExceptionalValue();
    }

}

