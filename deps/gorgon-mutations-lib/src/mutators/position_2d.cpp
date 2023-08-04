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

#include <position_2d.h>



// begin position2d Namespace

// Begin Position2D mutator
void Position2D::Position2DMutator::mutateRef(void* dataX, void* dataY)
{
    double* dataX_ptr = static_cast<double*>(dataX);
    double* dataY_ptr = static_cast<double*>(dataY);

    mutate(dataX_ptr, dataY_ptr);
}

// begin Gaussian Noise Mutator
Position2D::AnisotropicGaussianNoiseMutator::AnisotropicGaussianNoiseMutator(double meanX, double stdDevX,
        double meanY, double stdDevY)
    : m_xNoiseMutator(meanX, stdDevX)
    , m_yNoiseMutator(meanY, stdDevY)
{ }

Position2D::AnisotropicGaussianNoiseMutator::AnisotropicGaussianNoiseMutator(double meanX, double stdDevX,
        double meanY, double stdDevY, unsigned long seed)
    : m_xNoiseMutator(meanX, stdDevX, seed)
    , m_yNoiseMutator(meanY, stdDevY, seed+1)
    // Feed (seed+1) to yMutator so that it doesnt give same values as xMutator
{ }

// utilizes gaussianNoise delegative constructor for 0.0 mean
Position2D::AnisotropicGaussianNoiseMutator::AnisotropicGaussianNoiseMutator(double stdDevX, double stdDevY)
    : m_xNoiseMutator(stdDevX)
    , m_yNoiseMutator(stdDevY)
{ }

void Position2D::AnisotropicGaussianNoiseMutator::mutate(double* dataX, double* dataY)
{
    m_xNoiseMutator.mutate(dataX);
    m_yNoiseMutator.mutate(dataY);
}

void Position2D::AnisotropicGaussianNoiseMutator::mutateByReferenceSeeded(void* dataX, void* dataY, unsigned long seed)
{
    m_xNoiseMutator.mutateByReferenceSeeded(dataX, seed);
    m_yNoiseMutator.mutateByReferenceSeeded(dataY, seed);
}

// begin swapxy mutator
Position2D::SwapXYMutator::SwapXYMutator()
{ }

void Position2D::SwapXYMutator::mutate(double* dataX, double* dataY)
{
    double temp = *dataX;
    *dataX = *dataY;
    *dataY = temp;
}


// end position2D namespace
