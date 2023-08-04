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

/**
 * Checks that the GaussianNoiseMutator can be created and that it can generate exceptional values
 *
 */
TEST(GaussianNoiseMutator, AssessGetters)
{

    RecordProperty("summary", "Tests basic creation of two different GaussianNoiseMutators and if they can get basic values");

    const double inputMean = 1.0, inputStdDev = 1.5;

    //try basic constructor
    Double::GaussianNoiseMutator myTestMutator_twoInput(inputMean, inputStdDev);

    try {
        double noiseResult = myTestMutator_twoInput.getGaussianAdjustment();
        SUCCEED();
    } catch (...) {
        ADD_FAILURE();
    }

    double meanResult = myTestMutator_twoInput.getMean();
    double stdDevResult = myTestMutator_twoInput.getStdDev();

    EXPECT_EQ(meanResult, inputMean);
    EXPECT_EQ(stdDevResult, inputStdDev);


    // try 0.0 mean constructor
    Double::GaussianNoiseMutator myTestMutator_oneInput(inputStdDev);

    try {
        double noiseResult = myTestMutator_oneInput.getGaussianAdjustment();
        SUCCEED();
    } catch (...) {
        ADD_FAILURE();
    }

    meanResult = myTestMutator_oneInput.getMean();
    stdDevResult = myTestMutator_oneInput.getStdDev();

    EXPECT_EQ(meanResult, 0.0);
    EXPECT_EQ(stdDevResult, inputStdDev);
}

/**
 * Checks if the Gaussian adjusts a double value in the same memory
 *
 */
TEST(GaussianNoiseMutator, AssessMutate)
{
    RecordProperty("summary", "Tests GaussianNoiseMutator regular and seeded mutate");

    const double inputMean = 1.0, inputStdDev = 0.1, initialValue = 3.0;
    const unsigned long inputSeed = 0;


    Double::GaussianNoiseMutator myTestMutator(inputMean, inputStdDev, inputSeed);

    double inputVal = initialValue;
    double* testInput = &inputVal;

    myTestMutator.mutate(testInput);


    Double::GaussianNoiseMutator myBigTestMutator(10000, 1);

    inputVal = initialValue;
    testInput = &inputVal;

    myBigTestMutator.mutate(testInput);

    // check if Value is changed
    EXPECT_NE(*testInput, initialValue);
}

/**
 * Checks if the Gaussian adjusts seeded is deterministic
 *
 */
TEST(GaussianNoiseMutator, AssessMutateSeeded)
{
    RecordProperty("summary", "Tests GaussianNoiseMutator seeded mutate");

    const double inputMean = 1.0, inputStdDev = 0.1, initialValue = 3.0;
    const int numIterations = 100;
    const double expectationRange = 0.06;
    const unsigned long inputSeed = 0;

    Double::GaussianNoiseMutator myTestMutator(inputMean, inputStdDev, inputSeed);

    double inputVal = initialValue;
    double* testInput = &inputVal;

    myTestMutator.mutateByReferenceSeeded(testInput, inputSeed);

    double outputValue = *testInput;
    inputVal = initialValue;

    myTestMutator.mutateByReferenceSeeded(testInput, inputSeed);

    EXPECT_EQ(*testInput, outputValue)<< initialValue << ":" << (*testInput) << ":" << outputValue;

    // perform mutations multiple times and make sure
    // the mean is actually where the parameters set it
    // (within an epsilon tolerance)

    double sum = 0;

    // calculate the average
    for (int i = 0; i < numIterations; i++) {
        inputVal = initialValue;
        myTestMutator.mutateByReferenceSeeded(testInput, i);
        sum += inputVal;
    }

    double avg = sum / numIterations;

    double avgWithoutOffset = avg - inputMean;

    EXPECT_LT(abs(initialValue - avgWithoutOffset), expectationRange);

}

/**
 *  tests that the StuckValueMutator can be constructed
 *
 */
TEST(StuckValueMutator, AssessConstructor)
{
    RecordProperty("summary", "test creation of mutator");

    try {
        Double::StuckValueMutator myTestMutator(0.0);
        SUCCEED();
    } catch (...) {
        ADD_FAILURE();
    }
}

/**
 *  tests that the StuckValueMutator can mutate
 *
 */
TEST(StuckValueMutator, AssessMutator)
{
    RecordProperty("summary", "test mutate of mutator");

    const double initialValue = 0.0, stuckVal = 10.0;
    double inputVal = initialValue;
    double* testInput = &inputVal;

    Double::StuckValueMutator myTestMutator(stuckVal);

    myTestMutator.mutate(testInput);

    EXPECT_EQ(inputVal, stuckVal);
}

