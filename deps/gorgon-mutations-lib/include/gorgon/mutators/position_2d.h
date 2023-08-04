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

#ifndef __GORGON_MUTATORS_POSITION_2D_H__
#define __GORGON_MUTATORS_POSITION_2D_H__

#include "base.h"
#include "floats.h"


namespace Position2D {

/**
 * @brief A Parent superclass for position mutator types to inherit from
 *
 */
class Position2DMutator: public BaseMutator
{
public:

    /**
     * @brief Accepts some data, modifies it, then returns the data
     * @note derived classes should implement this function with desired functionality
     * @note this method can't be const because implementations like stuck value requires memory
     * @note this mutator does not implement the mutateRef method - extend it via middleware specific containers as needed
     *
     * @param dataX a pointer to the x value that is changed
     * @param dataY a pointer to the y value that is changed
     *
     * @returns None
     */
    virtual inline void mutate(double* dataX, double* dataY) {};

    /**
     * @brief takes a pointer to two data points and mitates them
     * @note derived classes do not need override this function - mutate is automatically called
     *
     * @param void* pointer to data
     *
     * @returns None
     */
    virtual void mutateRef(void* dataX, void* dataY);

    /**
     * @brief takes a pointer to two data points and mitates them Deterministically
     * @note Randomized derived classes should override this function.
     * Otherwise, the default mutate is called
     *
     * @param void* pointer to data
     *
     * @returns None
     */
    virtual inline void mutateByReferenceSeeded(void* dataX, void* dataY, unsigned long seed) {mutateRef(dataX, dataY);}

    /**
     * @brief make class pure-virtual by virtualizing destructor
     */
    virtual ~Position2DMutator() {};
};



/**
 * @brief Mutator that adjusts a given position in x and y in a gaussian distribution
 * @note Std Deviation and mean are provided at construction
 */
class AnisotropicGaussianNoiseMutator: public Position2DMutator
{

public:

    /**
     * @brief creates a mutator that will adjust given position's x and y
     *
     * @note this mutator does not implement the mutateRef method - extend it via middleware specific containers as needed
     *
     * @param meanX double mean for the x distribution (an offset)
     * @param stdDevX double standard deviation of x distribution
     * @param meanY double mean for the y distribution (an offset)
     * @param stdDevY double standard deviation of y distribution
     */
    AnisotropicGaussianNoiseMutator(double meanX, double stdDevX, double meanY, double stdDevY);

    /**
     * @brief creates a mutator that will adjust given position's x and y
     *
     * @note this mutator does not implement the mutateRef method - extend it via middleware specific containers as needed
     *
     * @param meanX double mean for the x distribution (an offset)
     * @param stdDevX double standard deviation of x distribution
     * @param meanY double mean for the y distribution (an offset)
     * @param stdDevY double standard deviation of y distribution
     * @param seed int used to seed randomness for mutator
     * 
     * @note yNoiseMutator is fed (seed+1) as its seed so that it doesn't produce
     *      the same results as xNoiseMutator
     */
    AnisotropicGaussianNoiseMutator(double meanX, double stdDevX, double meanY, double stdDevY, unsigned long seed);

    /**
     * @brief creates a mutator that will adjust given position's x and y
     *
     * @note the distribution assumes means of 0.
     *
     * @param stdDevX double standard deviation of x distribution
     * @param stdDevY double standard deviation of y distribution
     */
    AnisotropicGaussianNoiseMutator(double stdDevX, double stdDevY);


    /**
     * @brief takes a pointer to a position daa and randomly adjusts the x and y within a gaussian distribution
     *
     * @param dataX double pointer to x data which will be adusted
     * @param dataX double pointer to y data which will be adusted
     *
     * @return None
     */
    void mutate(double* dataX, double* dataY) override;

    /**
     * @brief takes a pointer to two data points and mutates them in a seeded, random gaussian
     * @note determinism is not guarunteed unless the Mutator is constructed with a specified seed as well.
     * 
     * @param dataX pointer to data representing X value
     * @param dataY pointer to data representing Y value
     * @param seed int used to seed any random generation
     *
     * @returns None
     */
    void mutateByReferenceSeeded(void* dataX, void* dataY, unsigned long seed) override;


private:

    /**@private
     * @brief gaussian mutator for x values
     */
    Double::GaussianNoiseMutator m_xNoiseMutator;

    /**@private
     * @brief gaussian mutator for y values
     */
    Double::GaussianNoiseMutator m_yNoiseMutator;


};


/**
 * @brief Mutator that switches x and y in a given position
 */
class SwapXYMutator: public Position2DMutator
{

public:

    /**
     * @brief creates a mutator that will swap given x and y
     *
     * @note this mutator does not implement the mutateRef method - extend it via middleware specific containers as needed
     */
    SwapXYMutator();

    /**
     * @brief takes a pointer to and swaps the x and y
     *
     * @param dataX double pointer to data which will be adusted
     * @param dataY double pointer to data which will be adusted
     *
     * @return None
     */
    void mutate(double* dataX, double* dataY) override;
};




}; // end Position2D namespace

#endif