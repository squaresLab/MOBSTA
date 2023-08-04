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

#include <nlohmann/json.hpp>
#include <gtest/gtest.h>
#include <gorgon/mutators/numeric.h>
#include <gorgon/utils.h>

template <typename T>
class NumericTests : public ::testing::Test
{};

using MyTypes = ::testing::Types<float, double, int, uint>;
TYPED_TEST_SUITE(NumericTests, MyTypes);

/**
 * Checks that the GaussianNoiseMutator can be created and that it can generate exceptional values
 *
 */
TYPED_TEST(NumericTests, GaussianNoiseMutatorAssessGetters)
{

    this->RecordProperty("summary", "Tests basic creation of two different GaussianNoiseMutators and if they can get basic values");

    const double inputMean = 1, inputStdDev = 1;

    //try basic constructor
    Numeric::GaussianNoiseMutator<TypeParam> myTestMutator_twoInput(inputMean, inputStdDev);

    try {
        TypeParam noiseResult = myTestMutator_twoInput.getGaussianAdjustment();
        SUCCEED();
    } catch (...) {
        ADD_FAILURE();
    }

    double meanResult = myTestMutator_twoInput.getMean();
    double stdDevResult = myTestMutator_twoInput.getStdDev();

    EXPECT_EQ(meanResult, inputMean);
    EXPECT_EQ(stdDevResult, inputStdDev);


    // try 0 mean constructor
    Numeric::GaussianNoiseMutator<TypeParam> myTestMutator_oneInput(inputStdDev);

    try {
        TypeParam noiseResult = myTestMutator_oneInput.getGaussianAdjustment();
        SUCCEED();
    } catch (...) {
        ADD_FAILURE();
    }

    meanResult = myTestMutator_oneInput.getMean();
    stdDevResult = myTestMutator_oneInput.getStdDev();

    EXPECT_EQ(meanResult, 0);
    EXPECT_EQ(stdDevResult, inputStdDev);
}

/**
 * Checks if the Gaussian adjusts a  value in the same memory
 *
 */
TYPED_TEST(NumericTests, GaussianNoiseMutatorAssessMutate)
{
    this->RecordProperty("summary", "Tests GaussianNoiseMutator regular and seeded mutate");

    const double inputMean = 1, inputStdDev = 1;
    const TypeParam initialValue = 3;
    const unsigned long inputSeed = 0;


    Numeric::GaussianNoiseMutator<TypeParam> myTestMutator(inputMean, inputStdDev, inputSeed);

    TypeParam inputVal = initialValue;
    TypeParam* testInput = &inputVal;

    myTestMutator.mutate(testInput);


    Numeric::GaussianNoiseMutator<TypeParam> myBigTestMutator(10000, 1);

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
TYPED_TEST(NumericTests, GaussianNoiseMutatorAssessMutateSeeded)
{
    this->RecordProperty("summary", "Tests GaussianNoiseMutator seeded mutate");

    const double inputMean = 10, inputStdDev = 1;
    const TypeParam initialValue = 3;
    const int numIterations = 100;
    const double expectationRange = 1.5;
    const unsigned long inputSeed = 0;

    Numeric::GaussianNoiseMutator<TypeParam> myTestMutator(inputMean, inputStdDev, inputSeed);

    TypeParam inputVal = initialValue;
    TypeParam* testInput = &inputVal;

    myTestMutator.mutateByReferenceSeeded(testInput, inputSeed);

    TypeParam outputValue = *testInput;
    inputVal = initialValue;

    myTestMutator.mutateByReferenceSeeded(testInput, inputSeed);

    EXPECT_EQ(*testInput, outputValue)<< initialValue << ":" << (*testInput) << ":" << outputValue;

    // perform mutations multiple times and make sure
    // the mean is actually where the parameters set it
    // (within an epsilon tolerance)

    TypeParam sum = 0;

    // calculate the average
    for (int i = 0; i < numIterations; i++) {
        inputVal = initialValue;
        myTestMutator.mutateByReferenceSeeded(testInput, i);
        sum += inputVal;
    }

    double avg = sum / numIterations;

    double avgWithoutOffset = avg - inputMean;

    double difference = (initialValue - avgWithoutOffset);

    EXPECT_LT(abs(difference), expectationRange);

}

/**
 *  tests that the StuckValueMutator can be constructed
 *
 */
TYPED_TEST(NumericTests, StuckValueMutatorAssessConstructor)
{
    this->RecordProperty("summary", "test creation of mutator");

    try {
        Numeric::StuckValueMutator<TypeParam> myTestMutator(0);
        SUCCEED();
    } catch (...) {
        ADD_FAILURE();
    }
}

/**
 *  tests that the StuckValueMutator can mutate
 *
 */
TYPED_TEST(NumericTests, StuckValueMutatorAssessMutator)
{
    this->RecordProperty("summary", "test mutate of mutator");

    const TypeParam initialValue = 0, stuckVal = 10;
    TypeParam inputVal = initialValue;
    TypeParam* testInput = &inputVal;

    Numeric::StuckValueMutator<TypeParam> myTestMutator(stuckVal);

    myTestMutator.mutate(testInput);

    EXPECT_EQ(inputVal, stuckVal);
}


/**
 * Checks that the GaussianNoiseMutator can be created and that it can generate exceptional values
 *
 */
TEST(NumericFactoryTest, CreateGaussianNoiseMutators)
{

    this->RecordProperty("summary", "Tests factory creation of GaussianNoiseMutators");

    nlohmann::json mutator_args;
    mutator_args["mean"] = 10000.0;
    mutator_args["stdDev"] = 1.0;

    utils::MutatorFactory fact;
    std::shared_ptr<BaseMutator> doubleMutator = fact.buildMutator("Double_GaussianNoiseMutator", mutator_args);
    const double doubleInitial = 10.0;
    double doubleData = doubleInitial;
    doubleMutator->mutateRef(&doubleData);
    EXPECT_LT(doubleInitial, doubleData);

    std::shared_ptr<BaseMutator> floatMutator = fact.buildMutator("Float32_GaussianNoiseMutator", mutator_args);
    const float floatInitial = 10.0;
    float floatData = floatInitial;
    floatMutator->mutateRef(&floatData);
    EXPECT_LT(floatInitial, floatData);

    std::shared_ptr<BaseMutator> intMutator = fact.buildMutator("Int32_GaussianNoiseMutator", mutator_args);
    const int intInitial = 10.0;
    int intData = intInitial;
    intMutator->mutateRef(&intData);
    EXPECT_LT(intInitial, intData);

    std::shared_ptr<BaseMutator> uintMutator = fact.buildMutator("UInt32_GaussianNoiseMutator", mutator_args);
    const uint uintInitial = 10.0;
    uint uintData = uintInitial;
    uintMutator->mutateRef(&uintData);
    EXPECT_LT(uintInitial, uintData);

}


/**
 * Checks that the StuckValueMutator can be created and that it can modify values
 *
 */
TEST(NumericFactoryTest, CreateStuckValueMutators)
{

    this->RecordProperty("summary", "Tests factory creation of StuckValueMutators");

    nlohmann::json mutator_args;
    mutator_args["stuckValue"] = 1.0;

    utils::MutatorFactory fact;
    std::shared_ptr<BaseMutator> doubleMutator = fact.buildMutator("Double_StuckValueMutator", mutator_args);
    const double doubleInitial = 10.0;
    double doubleData = doubleInitial;
    doubleMutator->mutateRef(&doubleData);
    EXPECT_EQ(doubleData, 1.0);

    std::shared_ptr<BaseMutator> floatMutator = fact.buildMutator("Float32_StuckValueMutator", mutator_args);
    const float floatInitial = 10.0;
    float floatData = floatInitial;
    floatMutator->mutateRef(&floatData);
    EXPECT_EQ(floatData, 1.0);

    std::shared_ptr<BaseMutator> intMutator = fact.buildMutator("Int32_StuckValueMutator", mutator_args);
    const int intInitial = 10.0;
    int intData = intInitial;
    intMutator->mutateRef(&intData);
    EXPECT_EQ(intData, 1.0);

    std::shared_ptr<BaseMutator> uintMutator = fact.buildMutator("UInt32_StuckValueMutator", mutator_args);
    const uint uintInitial = 10.0;
    uint uintData = uintInitial;
    uintMutator->mutateRef(&uintData);
    EXPECT_EQ(uintData, 1.0);

}