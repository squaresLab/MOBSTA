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

#include <gtest/gtest.h>
#include <gorgon/mutators/position_2d.h>

/**
 * Checks that the Isotropic GaussianNoiseMutator can be created
 *
 */
TEST(AnisotropicGaussianNoiseMutator, AssessConstructors)
{

    RecordProperty("summary", "Tests basic creation of two different AnisotropicGaussianNoiseMutators");

    const double inputMeanX = 1.0, inputStdDevX = 1.5, inputMeanY = 1.0, inputStdDevY = 1.5;;
    const unsigned long inputSeed = 1;

    try {
        //try basic constructor
        Position2D::AnisotropicGaussianNoiseMutator myTestMutator_fiveInput(inputMeanX, inputStdDevX,
                inputMeanY, inputStdDevY, inputSeed);
        SUCCEED();
    } catch (...) {
        ADD_FAILURE();
    }

    try {
        //try basic constructor
        Position2D::AnisotropicGaussianNoiseMutator myTestMutator_fourInput(inputMeanX, inputStdDevX,
                inputMeanY, inputStdDevY);
        SUCCEED();
    } catch (...) {
        ADD_FAILURE();
    }

    try {
        //try delegative constructor
        Position2D::AnisotropicGaussianNoiseMutator myTestMutator_twoInput(inputStdDevX, inputStdDevY);
        SUCCEED();
    } catch (...) {
        ADD_FAILURE();
    }
}


/**
 * Checks if the AnisotropicGaussian adjusts a double value in the same memory
 *
 */
TEST(AnisotropicGaussianNoiseMutator, AssessMutate)
{
    RecordProperty("summary", "Tests AnisotropicGaussianNoiseMutator mutate");

    const double inputMeanX = 1.0, inputStdDevX = 0.15, initialValueX = 3.0;
    const double inputMeanY = -1.0, inputStdDevY = 0.05, initialValueY = -3.0;
    const unsigned long inputSeed = 1;

    Position2D::AnisotropicGaussianNoiseMutator myTestMutator(inputMeanX, inputStdDevX, inputMeanY, inputStdDevY);

    double inputValX = initialValueX;
    double* testInputX = &inputValX;

    double inputValY = initialValueY;
    double* testInputY = &inputValY;

    myTestMutator.mutate(testInputX, testInputY);

    const double bigMean = 1000000, smallStdDev = 0.0001;
    Position2D::AnisotropicGaussianNoiseMutator myBigTestMutator(bigMean, smallStdDev, bigMean, smallStdDev);

    inputValX = initialValueX;
    testInputX = &inputValX;

    inputValY = initialValueY;
    testInputY = &inputValY;

    myBigTestMutator.mutate(testInputX, testInputY);

    // check if Value is changed
    // This test is non-deterministic, but vanishingly unlikely to fail
    EXPECT_NE(*testInputX, initialValueX);
    EXPECT_NE(*testInputY, initialValueY);
}

/**
 * Checks if the Gaussian adjusts seeded is deterministic
 *
 */
TEST(AnisotropicGaussianNoiseMutator, AssessMutateSeeded)
{
    RecordProperty("summary", "Tests AnisotropicGaussianNoiseMutator seeded mutate");

    const double inputMeanX = 1.0, inputStdDevX = 0.1, initialValueX = 3.0;
    const double inputMeanY = 1.0, inputStdDevY = 0.1, initialValueY = 3.0;
    const int numIterations = 100;
    const double expectationRange = 0.07;
    const unsigned long inputSeed = 0;

    Position2D::AnisotropicGaussianNoiseMutator myTestMutator(inputMeanX, inputStdDevX,
                                                            inputMeanY, inputStdDevY,
                                                            inputSeed);

    double inputValX = initialValueX;
    double* testInputX = &inputValX;

    double inputValY = initialValueY;
    double* testInputY = &inputValY;

    myTestMutator.mutateByReferenceSeeded(testInputX, testInputY, inputSeed);

    double outputValueX = *testInputX;
    inputValX = initialValueX;

    double outputValueY = *testInputY;
    inputValY = initialValueY;

    myTestMutator.mutateByReferenceSeeded(testInputX, testInputY, inputSeed);

    EXPECT_EQ(*testInputX, outputValueX)<< initialValueX << ":" << (*testInputX) << ":" << outputValueX;
    EXPECT_EQ(*testInputY, outputValueY)<< initialValueY << ":" << (*testInputY) << ":" << outputValueY;

    // perform mutations multiple times and make sure
    // the mean is actually where the parameters set it
    // (within an epsilon tolerance)

    double sumX = 0;
    double sumY = 0;

    // calculate the average
    for (int i = 0; i < numIterations; i++)
    {
        inputValX = initialValueX;
        inputValY = initialValueY;

        myTestMutator.mutateByReferenceSeeded(testInputX, testInputY, i);

        sumX += inputValX;
        sumY += inputValY;

    }

    double avgX = sumX / numIterations;
    double avgY = sumY / numIterations;

    double avgWithoutOffsetX = avgX - inputMeanX;
    double avgWithoutOffsetY = avgY - inputMeanY;

    EXPECT_LT(abs(initialValueX - avgWithoutOffsetX), expectationRange);
    EXPECT_LT(abs(initialValueY - avgWithoutOffsetY), expectationRange);
}

/**
 * Checks that the swapXYMutator can be created
 *
 */
TEST(SwapXYMutator, AssessConstructors)
{

    RecordProperty("summary", "Tests basic creation of SwapXYMutator");

    try {
        //try basic constructor
        Position2D::SwapXYMutator myTestMutator;
        SUCCEED();
    } catch (...) {
        ADD_FAILURE();
    }
}

/**
 * Checks that the swapXYMutator can mutate correctly
 *
 */
TEST(SwapXYMutator, AssessMutate)
{

    RecordProperty("summary", "Tests mutation of SwapXYMutator");

    Position2D::SwapXYMutator myTestMutator;

    const double initialX = 1.0, initialY = 2.0;

    double inputValX = initialX;
    double inputValY = initialY;

    double* testInputX = &inputValX;
    double* testInputY = &inputValY;

    myTestMutator.mutate(testInputX, testInputY);

    EXPECT_EQ(inputValX, initialY);
    EXPECT_EQ(inputValY, initialX);
}