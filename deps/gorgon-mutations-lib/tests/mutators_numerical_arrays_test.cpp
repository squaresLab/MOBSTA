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


#include <gtest/gtest.h>
#include <gorgon/mutators/numerical_arrays.h>

/**
 * Check that the ReplaceWholeArrayMutator works
 */ 
TEST(FixedSizeNumericalArray_ReplaceWholeArrayMutator, AssessMutate)
{
    RecordProperty("summary", "Tests ReplaceWholeArrayMutator");

    int replacementValue = 1;
    std::size_t arraySize = 10;
    FixedSizeNumericalArray::ReplaceWholeArrayMutator<int> myTestMutator(replacementValue, arraySize);

    // Create the array of test data
    std::vector<int> testArray(arraySize, 0);
    myTestMutator.mutate(testArray.data());
    for (std::size_t arrayIndex = 0; arrayIndex < testArray.size(); ++arrayIndex)
    {
        EXPECT_EQ(testArray[arrayIndex], replacementValue);
    }
}

TEST(FixedSizeNumericalArray_ProbabalisticReplaceMutator, AssessMutate)
{
    RecordProperty("summary", "Tests ProbabalisticReplaceMutator");

    int originalValue = 0;
    int replacementValue = 1;
    std::size_t arraySize = 100;
    const unsigned long inputSeed = 42;
    FixedSizeNumericalArray::ProbabalisticReplaceMutator<int> myTestMutator(replacementValue, 1.0, arraySize, inputSeed);

    // Create the array of test data
    std::vector<int> testArray(arraySize, originalValue);
    myTestMutator.mutateByReferenceSeeded(testArray.data(), inputSeed);

    for (std::size_t i = 0; i < testArray.size(); ++i)
    {
        EXPECT_EQ(testArray[i], replacementValue);
    }

    // Create another mutator with a 0% chance. Everything should stay the same
    FixedSizeNumericalArray::ProbabalisticReplaceMutator<int> myTestMutator2(originalValue, 0.0, arraySize, inputSeed);
    myTestMutator2.mutateByReferenceSeeded(testArray.data(), inputSeed);

    for (std::size_t i = 0; i < testArray.size(); ++i)
    {
        EXPECT_EQ(testArray[i], replacementValue);
    }

    // Now a 50-50 chance. About half of the values should change
    FixedSizeNumericalArray::ProbabalisticReplaceMutator<int> myTestMutator3(originalValue, 0.5, arraySize, inputSeed);
    myTestMutator3.mutateByReferenceSeeded(testArray.data(), inputSeed);

    int nValuesChanged = 0;
    for (std::size_t i = 0; i < testArray.size(); ++i)
    {
        if (testArray[i] == originalValue) ++nValuesChanged;
    }

    EXPECT_LT(nValuesChanged, 60);
    EXPECT_GT(nValuesChanged, 40);
}

/**
 * Check that the AddToWholeArrayMutator works with positive offset, and that it works with the array
 * min/max values
 */ 
TEST(FixedSizeNumericalArray_AddToWholeArrayMutator, AssessMutatePositive)
{
    RecordProperty("summary", "Tests AddToWholeArrayMutatorPositive");

    int arrayOffset = 10;
    int arrayMinValue = 0;
    int arrayMaxValue = 20;
    std::size_t arraySize = 10;
    FixedSizeNumericalArray::AddToWholeArrayMutator<int> myTestMutator(arrayOffset, arrayMinValue, arrayMaxValue, arraySize);

    // First test: offset without exceeding the maximum
    int initialArrayValue = 5;
    std::vector<int> testArray(arraySize, initialArrayValue);
    myTestMutator.mutate(testArray.data());
    for (std::size_t arrayIndex = 0; arrayIndex < testArray.size(); ++arrayIndex)
    {
        EXPECT_EQ(testArray[arrayIndex], initialArrayValue+arrayOffset);
    }

    // And again, this time make sure we don't go over the maximum
    myTestMutator.mutate(testArray.data());
    for (std::size_t arrayIndex = 0; arrayIndex < testArray.size(); ++arrayIndex)
    {
        EXPECT_EQ(testArray[arrayIndex], arrayMaxValue);
    }
}


/**
 * Check that the AddToWholeArrayMutator works with negative offset, and that it works with the array
 * min/max values
 */ 
TEST(FixedSizeNumericalArray_AddToWholeArrayMutator, AssessMutateNegative)
{
    RecordProperty("summary", "Tests AddToWholeArrayMutatorNegative");

    int arrayOffset = -10;
    int arrayMinValue = 0;
    int arrayMaxValue = 20;
    std::size_t arraySize = 10;
    FixedSizeNumericalArray::AddToWholeArrayMutator<int> myTestMutator(arrayOffset, arrayMinValue, arrayMaxValue, arraySize);

    // First test: offset without going under the minimum
    int initialArrayValue = 15;
    std::vector<int> testArray(arraySize, initialArrayValue);
    myTestMutator.mutate(testArray.data());
    for (std::size_t arrayIndex = 0; arrayIndex < testArray.size(); ++arrayIndex)
    {
        EXPECT_EQ(testArray[arrayIndex], initialArrayValue+arrayOffset);
    }

    // And again, this time make sure we don't go under the minimum
    myTestMutator.mutate(testArray.data());
    for (std::size_t arrayIndex = 0; arrayIndex < testArray.size(); ++arrayIndex)
    {
        EXPECT_EQ(testArray[arrayIndex], arrayMinValue);
    }
}