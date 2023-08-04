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


#include <gorgon_ros/mutators/strings.h>


// Begin StringMutator

void StringSerialized::StringMutator::mutateRef(void* data)
{
    std_msgs::String* data_ptr = static_cast<std_msgs::String*>(data);
    mutate(data_ptr);
}


void StringSerialized::StringMutator::mutateSerialized(void* data, size_t dataSize)
{
    // deserialize
    uint8_t* dataPointer = static_cast<uint8_t*>(data);
    ros::serialization::IStream desiredMsgStream(dataPointer, dataSize);
    std_msgs::String desiredMsg;
    ros::serialization::deserialize(desiredMsgStream, desiredMsg);

    mutate(&desiredMsg);

    // reserialize
    ros::serialization::OStream mutatedMsgStream(dataPointer, dataSize);
    ros::serialization::serialize(mutatedMsgStream, desiredMsg);
}


// begin ReplaceCharMutator


StringSerialized::ReplaceCharMutator::ReplaceCharMutator(char replacement, int index)
    : m_replaceChar(replacement, index)
{ }


void StringSerialized::ReplaceCharMutator::mutate(std_msgs::String* data)
{
    char* dataAsCharArray = static_cast<char*>(&(data->data[0]));
    m_replaceChar.mutate(dataAsCharArray, data->data.length());
    data->data = std::string(dataAsCharArray, data->data.length());
}