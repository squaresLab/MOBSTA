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

#include <iostream>
#include <gtest/gtest.h>
#include <gorgon/mutators/floats.h>

/**
 * Checks that the DictionaryAttack can be created, that it can generate exceptional values, and that it knows
 * if a value is exceptional
 *
 */
TEST(DictionaryAttackMutator, AssessGetExceptional)
{

    RecordProperty("summary", "Tests basic creation of a DictionaryAttackMutator and if it can get/check an exceptional value");

    Double::DictionaryAttackMutator myTestMutator;
    double result = myTestMutator.getExceptionalValue();

    EXPECT_TRUE(myTestMutator.valInExceptionalValuesArray(2.0));
    EXPECT_TRUE(myTestMutator.valInExceptionalValuesArray(result)) << result;

}

/**
 * Checks if the Dictionary Attack replaces a float value in the same memory
 *
 */
TEST(DictionaryAttackMutator, AssessMutate)
{
    RecordProperty("summary", "Tests DictionaryAttackMutator mutation");

    Double::DictionaryAttackMutator myTestMutator;

    //3.0 intentionally chosen as 0.0, 1.0, and 2.0 are possible exceptional values
    const double initialValue = 3.0;

    double inputVal = initialValue;
    double* testInput = &inputVal;

    myTestMutator.mutate(testInput);

    // make sure mutator changed the value
    EXPECT_NE(*testInput, initialValue);

    EXPECT_TRUE(myTestMutator.valInExceptionalValuesArray(*testInput)) << (*testInput);
}


/**
 * Checks if the Dictionary Attack is deterministic
 *
 */
TEST(DictionaryAttackMutator, AssessMutateSeeded)
{
    RecordProperty("summary", "Tests DictionaryAttackMutator deterministic mutation");


    const unsigned long inputSeed = 1;

    Double::DictionaryAttackMutator myTestMutator(inputSeed);

    //3.0 intentionally chosen as 0.0, 1.0, and 2.0 are possible exceptional values
    const double initialValue = 3.0;

    double inputVal = initialValue;
    double* testInput = &inputVal;

    myTestMutator.mutateByReferenceSeeded(testInput, inputSeed);

    // make sure mutator changed the value
    EXPECT_NE(*testInput, initialValue);

    EXPECT_TRUE(myTestMutator.valInExceptionalValuesArray(*testInput));

    const double outputValue = *testInput;
    inputVal = initialValue;

    myTestMutator.mutateByReferenceSeeded(testInput, inputSeed);

    EXPECT_EQ(*testInput, outputValue);

}
