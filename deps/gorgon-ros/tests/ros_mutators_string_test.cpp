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


#include <cstdint>
#include <gtest/gtest.h>
#include <std_msgs/String.h>
#include <gorgon_ros/mutators/strings.h>

/**
 * Checks that the ReplaceCharMutator can be created
 *
 */
TEST(StringSerialized_ReplaceCharMutator, AssessConstructor)
{

    RecordProperty("summary", "Tests basic creation of a ReplaceCharMutator");

    StringSerialized::ReplaceCharMutator myTestMutator('a', 0);

    SUCCEED();
}

/**
 * Checks if the ReplaceCharacterMutator replaces a character in the same memory correctly
 *
 */
TEST(StringSerialized_ReplaceCharMutator, AssessMutate)
{
    RecordProperty("summary", "Tests ReplaceCharMutator mutation");

    // setup mutator
    const char testReplacmentChar = 'z';
    const int testReplacmentIndex = 5;

    StringSerialized::ReplaceCharMutator myTestMutator(testReplacmentChar, testReplacmentIndex);

    // setup serialized string data
    std_msgs::String input;
    std::string initialString = "abcdefg";
    const size_t strLength = initialString.length();
    input.data = initialString;

    myTestMutator.mutate(&input);


    // test that string size is same
    EXPECT_EQ(input.data.length(), strLength);

    // test that string is same but with replaced character
    for (uint32_t i = 0; i < strLength; i++) {
        if (i == testReplacmentIndex) {
            EXPECT_EQ(input.data[i], testReplacmentChar);
        } else {
            EXPECT_EQ(input.data[i], initialString[i]);
        }

    }

    // test mutator going over index
    // setup smaller serialized string data
    std_msgs::String shorterInput;
    std::string shortInitialString = "abc";
    shorterInput.data = shortInitialString;
    const uint32_t shorterStrLength = shortInitialString.length();

    myTestMutator.mutate(&shorterInput);

    // test that string size is same
    EXPECT_EQ(shorterInput.data.length(), shorterStrLength);

    // test that string is same
    for (uint32_t i = 0; i < shorterStrLength; i++) {
        EXPECT_EQ(shorterInput.data[i], shortInitialString[i]);
    }


}