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
#include <gorgon/mutators/char_arrays.h>

/**
 * Checks that the ReplaceCharMutator can be created
 *
 */
TEST(ReplaceCharMutator, AssessConstructor)
{

    RecordProperty("summary", "Tests basic creation of a ReplaceCharMutator");

    CharArray::ReplaceCharMutator myTestMutator('a', 0);

    SUCCEED();
}

/**
 * Checks if the ReplaceCharacterMutator replaces a character in the same memory correctly
 *
 */
TEST(ReplaceCharMutator, AssessMutate)
{
    RecordProperty("summary", "Tests ReplaceCharMutator mutation");

    // setup mutator
    const char testReplacmentChar = 'z';
    const int testReplacmentIndex = 5;

    CharArray::ReplaceCharMutator myTestMutator(testReplacmentChar, testReplacmentIndex);   

    // setup serialized string data
    const std::size_t arrLength = 7;
    char initialCharArray[] = {'a', 'b', 'c', 'd', 'e', 'f'};
    char* arrayGoalData = &(initialCharArray[0]);

    myTestMutator.mutate(arrayGoalData, arrLength);

    // test that array is same but with replaced character
    for (uint32_t i = 0; i < arrLength; i++)
    {
        if(i == testReplacmentIndex)
        {
            EXPECT_EQ(arrayGoalData[i], testReplacmentChar);
        }
        else
        {
            EXPECT_EQ(arrayGoalData[i], initialCharArray[i]);
        }

    }

    // test mutator going over index
    // setup smaller serialized string data
    const std::size_t arrLength2 = 3;
    char secondCharArray[] = {'a', 'b', 'c'};
    arrayGoalData = &(secondCharArray[0]);

    myTestMutator.mutate(arrayGoalData, arrLength2);

    // test that array is same but with replaced character
    for (uint32_t i = 0; i < arrLength2; i++)
    {
        EXPECT_EQ(arrayGoalData[i], initialCharArray[i]);
    }


}

