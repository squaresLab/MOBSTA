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
#include <gorgon_ros/utils.h>
#include <gorgon_ros/mutators/ros_base.h>
#include <nlohmann/json.hpp>
#include <geometry_msgs/Point.h>
using json = nlohmann::json;

/**
 * Tests the Ros MutatorFactory building
 */
TEST(RosMutatorFactory, AssessBuild)
{
    RecordProperty("summary", "Tests construction and use of RosMutatorFactory");

    const double initialZ = 0.0, mutatorSetZValue = 2.0;
    geometry_msgs::Point inputPoint;
    inputPoint.z = initialZ;

    geometry_msgs::Point* testInput = &inputPoint;

    json newMutatorArgs;
    newMutatorArgs["stuckZVal"] = mutatorSetZValue;
    std::string newMutatorType = "PointMsg_StuckZMutator";

    utils::RosMutatorFactory factory;
    std::shared_ptr<RosBaseMutator> MyMutatorP = factory.buildMutator(newMutatorType, newMutatorArgs);

    MyMutatorP->mutateRef(testInput);

    EXPECT_EQ(testInput->z, mutatorSetZValue);

}

/**
 * Tests the Ros MutatorFactory building and use of the mutateSerialized method
 */
TEST(RosMutatorFactory, AssessSerialized)
{
    RecordProperty("summary", "Tests RosMutatorFactory output with serialized data");

    const double initialZ = 0.0, mutatorSetZValue = 2.0;
    geometry_msgs::Point inputPoint;
    inputPoint.z = initialZ;

    geometry_msgs::Point* testInput = &inputPoint;

    json newMutatorArgs;
    newMutatorArgs["stuckZVal"] = mutatorSetZValue;
    std::string newMutatorType = "PointMsg_StuckZMutator";

    utils::RosMutatorFactory factory;
    std::shared_ptr<RosBaseMutator> MyMutatorP = factory.buildMutator(newMutatorType, newMutatorArgs);

    // serialize
    size_t serialSize = ros::serialization::serializationLength(*testInput);
    uint8_t dataBuffer[serialSize];
    uint8_t* dataPointer = &dataBuffer[0];

    ros::serialization::OStream inputMsgStream(dataPointer, serialSize);
    ros::serialization::serialize(inputMsgStream, *testInput);


    MyMutatorP->mutateSerialized(dataPointer, serialSize);

    // deserialize
    ros::serialization::IStream outputMsgStream(dataPointer, serialSize);
    ros::serialization::deserialize(outputMsgStream, *testInput);

    EXPECT_EQ(testInput->z, mutatorSetZValue);
}

/**
 * Tests the Ros MutatorFactory building and use of the mutateSerializedSeeded method
 */
TEST(RosMutatorFactory, AssessSerializedSeeded)
{
    RecordProperty("summary", "Tests RosMutatorFactory output with seeded serialized data");

    const double inputStdDevX = 0.1, inputStdDevY = .1, inputMeanX = 5.0, inputMeanY = -5.0;
    const unsigned long inputSeed = 1;

    const double initialX = 3.0, initialY = 4.0;
    geometry_msgs::Point inputPoint;
    inputPoint.x = initialX;
    inputPoint.y = initialY;
    geometry_msgs::Point* testInput = &inputPoint;

    json newMutatorArgs;
    newMutatorArgs["stdDevX"] = inputStdDevX;
    newMutatorArgs["stdDevY"] = inputStdDevY;
    newMutatorArgs["meanX"] = inputMeanX;
    newMutatorArgs["meanY"] = inputMeanY;
    std::string newMutatorType = "PointMsg_AnisotropicGaussianNoiseMutator";

    utils::RosMutatorFactory factory;
    std::shared_ptr<RosBaseMutator> MyMutatorP = factory.buildMutator(newMutatorType, newMutatorArgs, inputSeed);

    // serialize
    size_t serialSize = ros::serialization::serializationLength(*testInput);
    uint8_t dataBuffer[serialSize];
    uint8_t* dataPointer = &dataBuffer[0];

    ros::serialization::OStream inputMsgStream(dataPointer, serialSize);
    ros::serialization::serialize(inputMsgStream, *testInput);


    MyMutatorP->mutateSerializedSeeded(dataPointer, serialSize, inputSeed);

    // deserialize
    ros::serialization::IStream outputMsgStream(dataPointer, serialSize);
    ros::serialization::deserialize(outputMsgStream, *testInput);

    EXPECT_NE(testInput->x, initialX);
    EXPECT_NE(testInput->y, initialY);

    geometry_msgs::Point outputPoint;
    outputPoint.x = testInput->x;
    outputPoint.y = testInput->y;

    testInput->x = initialX;
    testInput->y = initialY;

    ros::serialization::OStream inputMsgStream2(dataPointer, serialSize);
    ros::serialization::serialize(inputMsgStream2, *testInput);


    MyMutatorP->mutateSerializedSeeded(dataPointer, serialSize, inputSeed);

    // deserialize
    ros::serialization::IStream outputMsgStream2(dataPointer, serialSize);
    ros::serialization::deserialize(outputMsgStream2, *testInput);

    EXPECT_EQ(testInput->x, outputPoint.x);
    EXPECT_EQ(testInput->y, outputPoint.y);

}

/**
 * Tests the Ros MutatorFactory building multiple mutators to test for possible memory leak
 * Just makes sure no errors or segfaults happen with a large number of active mutators
 * Previous versions would leak memory with some mu
 */
TEST(RosMutatorFactory, RegressionMemoryLeakOnMultipleMutators)
{
    RecordProperty("summary", "Tests RosMutatorFactory making many mutators to make sure no memory leaks break it");

    const double inputStdDevX = 0.1, inputStdDevY = .1, inputMeanX = 5.0, inputMeanY = -5.0;
    const unsigned long inputSeed = 1;
    const int numMutators = 100;

    const double initialX = 3.0, initialY = 4.0;
    geometry_msgs::Point inputPoint;
    inputPoint.x = initialX;
    inputPoint.y = initialY;
    geometry_msgs::Point* testInput = &inputPoint;

    json newMutatorArgs;
    newMutatorArgs["stdDevX"] = inputStdDevX;
    newMutatorArgs["stdDevY"] = inputStdDevY;
    newMutatorArgs["meanX"] = inputMeanX;
    newMutatorArgs["meanY"] = inputMeanY;
    std::string newMutatorType = "PointMsg_AnisotropicGaussianNoiseMutator";

    utils::RosMutatorFactory factory;
    std::vector<std::shared_ptr<RosBaseMutator>> mutatorContainer;

    for (int i = 0; i < numMutators; i++)
    {
        mutatorContainer.push_back(factory.buildMutator(newMutatorType, newMutatorArgs, inputSeed));
    }

    for (int i = 0; i < numMutators; i++)
    {
        mutatorContainer.at(i)->mutateByReferenceSeeded(testInput, inputSeed);
    }

    SUCCEED();
}

/**
 * Tests the Ros MutatorFactory building and use of Default gorgon
 */
TEST(RosMutatorFactory, AssessDefaultGorgon)
{
    RecordProperty("summary", "Tests RosMutatorFactory output with default Gorgon");

    const double initialX = 0.0, mutatorSetValue = 2.0;
    double inputVal = initialX;
    double* input = &inputVal;

    json newMutatorArgs;
    newMutatorArgs["stuckValue"] = mutatorSetValue;
    std::string newMutatorType = "Double_StuckValueMutator";

    utils::RosMutatorFactory factory;
    std::shared_ptr<RosBaseMutator> MyMutatorP = factory.buildMutator(newMutatorType, newMutatorArgs);

    MyMutatorP->mutateRef(input);

    EXPECT_EQ(inputVal, mutatorSetValue);

    inputVal = initialX;

    MyMutatorP->mutateSerialized(input, sizeof(inputVal));

    EXPECT_EQ(inputVal, mutatorSetValue);
}