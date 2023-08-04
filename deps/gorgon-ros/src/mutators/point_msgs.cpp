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

#include <gorgon_ros/mutators/point_msgs.h>

// Begin PointMsgMutator
void PointMsg::PointMsgMutator::mutateRef(void* data)
{
    geometry_msgs::Point* data_ptr = static_cast<geometry_msgs::Point*>(data);
    mutate(data_ptr);
}

void PointMsg::PointMsgMutator::mutateSerialized(void* data, size_t dataSize)
{
    // deserialize
    uint8_t* dataPointer = static_cast<uint8_t*>(data);
    ros::serialization::IStream desiredMsgStream(dataPointer, dataSize);
    geometry_msgs::Point desiredMsg;
    ros::serialization::deserialize(desiredMsgStream, desiredMsg);

    mutate(&desiredMsg);

    // reserialize
    ros::serialization::OStream mutatedMsgStream(dataPointer, dataSize);
    ros::serialization::serialize(mutatedMsgStream, desiredMsg);
}


// begin Point2d Namespace

// begin Gaussian Noise Mutator
PointMsg::AnisotropicGaussianNoiseMutator::AnisotropicGaussianNoiseMutator(double meanX, double stdDevX,
        double meanY, double stdDevY)
    : m_anisoGaussMutator(meanX, stdDevX, meanY, stdDevY)
{ }

PointMsg::AnisotropicGaussianNoiseMutator::AnisotropicGaussianNoiseMutator(double meanX, double stdDevX,
        double meanY, double stdDevY, unsigned long seed)
    : m_anisoGaussMutator(meanX, stdDevX, meanY, stdDevY, seed)
{ }

// utilizes gaussianNoise delegative constructor for 0.0 mean
PointMsg::AnisotropicGaussianNoiseMutator::AnisotropicGaussianNoiseMutator(double stdDevX, double stdDevY)
    : m_anisoGaussMutator(stdDevX, stdDevY)
{ }

void PointMsg::AnisotropicGaussianNoiseMutator::mutate(geometry_msgs::Point* data)
{
    m_anisoGaussMutator.mutate(&(data->x), &(data->y));
}

void PointMsg::AnisotropicGaussianNoiseMutator::mutateByReferenceSeeded(void* data, unsigned long seed)
{
    geometry_msgs::Point* data_ptr = static_cast<geometry_msgs::Point*>(data);
    m_anisoGaussMutator.mutateByReferenceSeeded(&(data_ptr->x), &(data_ptr->y), seed);
}

void PointMsg::AnisotropicGaussianNoiseMutator::mutateSerializedSeeded(void* data, size_t dataSize, unsigned long seed)
{
    // deserialize
    uint8_t* dataPointer = static_cast<uint8_t*>(data);
    ros::serialization::IStream desiredMsgStream(dataPointer, dataSize);
    geometry_msgs::Point desiredMsg;
    ros::serialization::deserialize(desiredMsgStream, desiredMsg);

    mutateByReferenceSeeded(&desiredMsg, seed);

    // reserialize
    ros::serialization::OStream mutatedMsgStream(dataPointer, dataSize);
    ros::serialization::serialize(mutatedMsgStream, desiredMsg);
}


// begin swapxy mutator
PointMsg::SwapXYMutator::SwapXYMutator()
    : m_swapMutator()
{ }

void PointMsg::SwapXYMutator::mutate(geometry_msgs::Point* data)
{
    m_swapMutator.mutate(&(data->x), &(data->y));
}

// begin stuckZ mutator
PointMsg::StuckZMutator::StuckZMutator(double setZValue)
    : m_stuckMutator(setZValue)
{ }

void PointMsg::StuckZMutator::mutate(geometry_msgs::Point* data)
{
    m_stuckMutator.mutate(&(data->z));
}


// end Point2d namespace
