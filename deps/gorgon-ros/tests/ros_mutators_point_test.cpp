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
#include <gorgon_ros/mutators/point_msgs.h>
#include <geometry_msgs/Point.h>


/**
 * Checks that the PointMsg::AnisotropicGaussianNoiseMutator can be created and that it can generate exceptional values
 *
 */
TEST(PointMsg_AnisotropicGaussianNoiseMutator, AssessGetters)
{

    RecordProperty("summary", "Tests basic creation of two different PointMsg GaussianNoiseMutators and if they can get basic values");

    const double inputStdDevX = 1.0, inputStdDevY = 2.0;
    const unsigned long inputSeed = 1;

    try {
        // try constructor
        PointMsg::AnisotropicGaussianNoiseMutator myTestMutator_twoInput(inputStdDevX, inputStdDevY);
        SUCCEED();
    } catch (...) {
        ADD_FAILURE();
    }

    const double meanX = 1.0, meanY = -1.0;

    try {
        // try constructor
        PointMsg::AnisotropicGaussianNoiseMutator myTestMutator_fourInput(meanX, inputStdDevX, meanY, inputStdDevY);
        SUCCEED();
    } catch (...) {
        ADD_FAILURE();
    }

    try {
        //try basic constructor
        PointMsg::AnisotropicGaussianNoiseMutator myTestMutator_fiveInput(meanX, inputStdDevX,
                meanY, inputStdDevY, inputSeed);
        SUCCEED();
    } catch (...) {
        ADD_FAILURE();
    }

}

/**
 * Checks if the Gaussian adjusts a double value in the same memory
 *
 */
TEST(PointMsg_AnisotropicGaussianNoiseMutator, AssessMutate)
{
    RecordProperty("summary", "Tests PointMsg AnisotropicGaussianNoiseMutator mutate");

    const double inputStdDevX = 0.1, inputStdDevY = .1, inputMeanX = 5.0, inputMeanY = -5.0;
    const double initialX = 3.0, initialY = 4.0;
    geometry_msgs::Point inputPoint;
    inputPoint.x = initialX;
    inputPoint.y = initialY;

    geometry_msgs::Point* testInput = &inputPoint;


    PointMsg::AnisotropicGaussianNoiseMutator myTestMutator(inputMeanX, inputStdDevX, inputMeanY, inputStdDevY);


    myTestMutator.mutate(testInput);

    double smallStdDev = 1;
    double bigMean = 1000000;
    PointMsg::AnisotropicGaussianNoiseMutator myBigTestMutator(bigMean, smallStdDev, bigMean, smallStdDev);
    inputPoint.x = initialX;
    inputPoint.y = initialY;
    testInput = &inputPoint;

    myBigTestMutator.mutate(testInput);

    // check if Value is changed
    // This test is non-deterministic, but vanishingly unlikely to fail
    EXPECT_NE(testInput->x, initialX);
    EXPECT_NE(testInput->y, initialY);
}


/**
 *
 */
TEST(PointMsg_AnisotropicGaussianNoiseMutator, AssessMutateSeeded)
{ 
    RecordProperty("summary", "Tests AnisotropicGaussianNoiseMutator seeded mutate");
    const double inputStdDevX = 0.1, inputStdDevY = .1, inputMeanX = 5.0, inputMeanY = -5.0;
    const int numIterations = 100;
    const double expectationRange = 0.07;
    const unsigned long inputSeed = 1;

    const double initialX = 3.0, initialY = 4.0;
    geometry_msgs::Point inputPoint;
    inputPoint.x = initialX;
    inputPoint.y = initialY;
    geometry_msgs::Point* testInput = &inputPoint;

    PointMsg::AnisotropicGaussianNoiseMutator myTestMutator(inputMeanX, inputStdDevX,
                                                            inputMeanY, inputStdDevY,
                                                            inputSeed);

    double inputValX = initialX;
    double* testInputX = &inputValX;

    double inputValY = initialY;
    double* testInputY = &inputValY;

    myTestMutator.mutateByReferenceSeeded(testInput, inputSeed);

    geometry_msgs::Point outputPoint;
    outputPoint.x = testInput->x;
    outputPoint.y = testInput->y;

    testInput->x = initialX;
    testInput->y = initialY;

    myTestMutator.mutateByReferenceSeeded(testInput, inputSeed);

    EXPECT_EQ(testInput->x, outputPoint.x);
    EXPECT_EQ(testInput->y, outputPoint.y);
}

/**
 * Regression test making sure the point msg doesn't swap the values 
 */
TEST(PointMsg_AnisotropicGaussianNoiseMutator, RegressionSwapXY)
{
    RecordProperty("summary", "Tests that the mutator doesnt swap x and y in mutation");

    // with these means, x should be less than 0 and y should be greater than 0
    const double inputStdDevX = 0.1, inputStdDevY = 0.1, inputMeanX = -10.0, inputMeanY = 10.0;
    const unsigned long inputSeed = 1;

    const double initialX = 0.0, initialY = 0.0;
    geometry_msgs::Point inputPoint;
    inputPoint.x = initialX;
    inputPoint.y = initialY;
    geometry_msgs::Point* testInput = &inputPoint;

    PointMsg::AnisotropicGaussianNoiseMutator myTestMutator(inputMeanX, inputStdDevX,
                                                            inputMeanY, inputStdDevY,
                                                            inputSeed);

    myTestMutator.mutateByReferenceSeeded(testInput, inputSeed);

    EXPECT_LT(inputPoint.x, 0);
    EXPECT_GT(inputPoint.y, 0);
}

/**
 * Tests creation and mutation of SwapXYMutator
 */
TEST(PointMsg_SwapXYMutator, AssessMutate)
{
    RecordProperty("summary", "Tests creation and mutation of SwapXYMutator");

    const double initialX = 1.0, initialY = 2.0;
    geometry_msgs::Point inputPoint;
    inputPoint.x = initialX;
    inputPoint.y = initialY;

    geometry_msgs::Point* testInput = &inputPoint;

    PointMsg::SwapXYMutator myTestMutator;

    myTestMutator.mutate(testInput);

    EXPECT_EQ(testInput->x, initialY);
    EXPECT_EQ(testInput->y, initialX);
}


/**
 * Tests creation and mutation of StuckZMutator
 */
TEST(PointMsg_StuckZMutator, AssessMutate)
{
    RecordProperty("summary", "Tests creation and mutation of StuckZMutator");

    const double initialZ = 0.0, mutatorSetZValue = 2.0;
    geometry_msgs::Point inputPoint;
    inputPoint.z = initialZ;

    geometry_msgs::Point* testInput = &inputPoint;

    PointMsg::StuckZMutator myTestMutator(mutatorSetZValue);

    myTestMutator.mutate(testInput);

    EXPECT_EQ(testInput->z, mutatorSetZValue);
}

