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
#include <gorgon/mutators/numeric_arrays.h>

template <typename T>
class FixedSizeNumericArrayTests : public ::testing::Test
{};

using MyTypes = ::testing::Types<float, double, int, uint>;
TYPED_TEST_SUITE(FixedSizeNumericArrayTests, MyTypes);


/**
 * Check that the ReplaceWholeArrayMutator works
 */ 
TYPED_TEST(FixedSizeNumericArrayTests, ReplaceWholeArrayMutatorAssessMutate)
{
    this->RecordProperty("summary", "Tests ReplaceWholeArrayMutator");

    TypeParam replacementValue = 1;
    std::size_t arraySize = 10;
    FixedSizeNumericArray::ReplaceWholeArrayMutator<TypeParam> myTestMutator(replacementValue, arraySize);

    // Create the array of test data
    std::vector<TypeParam> testArray(arraySize, 0);
    myTestMutator.mutate(testArray.data());
    for (std::size_t arrayIndex = 0; arrayIndex < testArray.size(); ++arrayIndex)
    {
        EXPECT_EQ(testArray[arrayIndex], replacementValue);
    }
}

TYPED_TEST(FixedSizeNumericArrayTests, ProbabalisticReplaceMutatorAssessMutate)
{
    this->RecordProperty("summary", "Tests ProbabalisticReplaceMutator");

    TypeParam originalValue = 0;
    TypeParam replacementValue = 1;
    std::size_t arraySize = 100;
    const unsigned long inputSeed = 42;
    FixedSizeNumericArray::ProbabalisticReplaceMutator<TypeParam> myTestMutator(replacementValue, 1.0, arraySize, inputSeed);

    // Create the array of test data
    std::vector<TypeParam> testArray(arraySize, originalValue);
    myTestMutator.mutateByReferenceSeeded(testArray.data(), inputSeed);

    for (std::size_t i = 0; i < testArray.size(); ++i)
    {
        EXPECT_EQ(testArray[i], replacementValue);
    }

    // Create another mutator with a 0% chance. Everything should stay the same
    FixedSizeNumericArray::ProbabalisticReplaceMutator<TypeParam> myTestMutator2(originalValue, 0.0, arraySize, inputSeed);
    myTestMutator2.mutateByReferenceSeeded(testArray.data(), inputSeed);

    for (std::size_t i = 0; i < testArray.size(); ++i)
    {
        EXPECT_EQ(testArray[i], replacementValue);
    }

    // Now a 50-50 chance. About half of the values should change
    FixedSizeNumericArray::ProbabalisticReplaceMutator<TypeParam> myTestMutator3(originalValue, 0.5, arraySize, inputSeed);
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
TYPED_TEST(FixedSizeNumericArrayTests, AddToWholeArrayMutatorAssessMutatePositive)
{
    this->RecordProperty("summary", "Tests AddToWholeArrayMutatorPositive");

    double arrayOffset = 10;
    TypeParam arrayMinValue = 0;
    TypeParam arrayMaxValue = 20;
    std::size_t arraySize = 10;
    FixedSizeNumericArray::AddToWholeArrayMutator<TypeParam> myTestMutator(arrayOffset, arrayMinValue, arrayMaxValue, arraySize);

    // First test: offset without exceeding the maximum
    TypeParam initialArrayValue = 5;
    std::vector<TypeParam> testArray(arraySize, initialArrayValue);
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
TYPED_TEST(FixedSizeNumericArrayTests, AddToWholeArrayMutatorAssessMutateNegative)
{
    this->RecordProperty("summary", "Tests AddToWholeArrayMutatorNegative");

    double arrayOffset = -10;
    TypeParam arrayMinValue = 0;
    TypeParam arrayMaxValue = 20;
    std::size_t arraySize = 10;
    FixedSizeNumericArray::AddToWholeArrayMutator<TypeParam> myTestMutator(arrayOffset, arrayMinValue, arrayMaxValue, arraySize);

    // First test: offset without going under the minimum
    TypeParam initialArrayValue = 15;
    std::vector<TypeParam> testArray(arraySize, initialArrayValue);
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