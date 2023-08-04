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

#ifndef __GORGON_ROS_MUTATORS_POINT_MSGS_H__
#define __GORGON_ROS_MUTATORS_POINT_MSGS_H__

#include <ros/serialization.h>
#include "geometry_msgs/Point.h"
#include <gorgon_ros/mutators/ros_base.h>
#include "gorgon/mutators/floats.h"
#include "gorgon/mutators/position_2d.h"


namespace PointMsg
{

    /**
     * @brief A Parent superclass for ROS Point msg Mutator types to inherit from
     *
     */
    class PointMsgMutator: public RosBaseMutator
    {
        public:

            /**
             * @brief Accepts some data, modifies it, then returns the data
             * @note derived classes should implement this function with desired functionality
             * @note this method can't be const because implementations like stuck value requires memory
             *
             * @param data a pointer to the Point msg that is changed
             *
             * @returns None
             */
            virtual inline void mutate(geometry_msgs::Point* data) =0;


            /**
             * @brief takes pointer to a point message and mutates it
             *
             * @param data pointer to expected data
             *
             */
            void mutateRef(void* data) override;

            /**
             * @brief takes a pointer to serialized point data
             * @note derived classes do not need override this function - mutate is automatically called
             *
             * @param data pointer to data
             * @param dataSize size in bytes of the data
             *
             * @returns None
             */
            void mutateSerialized(void* data, size_t dataSize) override;
    };



    /**
     * @brief Mutator that adjusts a given Point in x and y in a gaussian distribution
     * @note Std Deviation and mean are provided at construction
     */
    class AnisotropicGaussianNoiseMutator: public PointMsgMutator
    {

        public:

            /**
             * @brief creates a mutator that will adjust given Pointmsg's x and y
             *
             * @param meanX double mean for the x distribution (an offset)
             * @param stdDevX double standard deviation of x distribution
             * @param meanY double mean for the y distribution (an offset)
             * @param stdDevY double standard deviation of y distribution
             */
            AnisotropicGaussianNoiseMutator(double meanX, double stdDevX, 
                                          double meanY, double stdDevY);

            /**
             * @brief creates a mutator that will adjust given Pointmsg's x and y
             *
             * @param meanX double mean for the x distribution (an offset)
             * @param stdDevX double standard deviation of x distribution
             * @param meanY double mean for the y distribution (an offset)
             * @param stdDevY double standard deviation of y distribution
             * @param seed int used to seed mutator randomness
             */
            AnisotropicGaussianNoiseMutator(double meanX, double stdDevX, 
                                          double meanY, double stdDevY, 
                                          unsigned long seed);

            /**
             * @brief creates a mutator that will adjust given Pointmsg's x and y
             *
             * @note the distribution assumes means of 0.
             *
             * @param stdDevX double standard deviation of x distribution
             * @param stdDevY double standard deviation of y distribution
             */
            AnisotropicGaussianNoiseMutator(double stdDevX, double stdDevY);


            /**
             * @brief takes a pointer to a Pointmsg and randomly adjusts the x and y within a gaussian distribution
             *
             * @param data double pointer to data which will be adusted
             *
             * @return None
             */
            void mutate(geometry_msgs::Point* data) override;

            /**
             * @brief takes a pointer to a PointMsg and mutates x and Y in a seeded, random gaussian
             * @note determinism is not guarunteed unless the Mutator is constructed with a specified seed as well.
             * 
             * @param data pointer to PointMsg
             * @param seed int used to seed any random generation
             *
             * @returns None
             */
            void mutateByReferenceSeeded(void* data, unsigned long seed);

            /**
             * @brief takes a pointer to serialized point data and mutates it deterministically
             *
             * @param data pointer to sertialized data
             * @param dataSize size in bytes of the data
             * @param seed int used to seed mutator randomness
             *
             * @returns None
             */
            void mutateSerializedSeeded(void* data, size_t dataSize, unsigned long seed);

        private:

            /**@private
             * @brief position2d mutator which is used to to all the mutation
             */
            Position2D::AnisotropicGaussianNoiseMutator m_anisoGaussMutator;

    };


    /**
     * @brief Mutator that switches x and y in a given Point
     */
    class SwapXYMutator: public PointMsgMutator
    {

        public:

            /**
             * @brief creates a mutator that will swap given Pointmsg's x and y
             *
             */
            SwapXYMutator();

            /**
             * @brief takes a pointer to a Pointmsg and swaps the x and y
             *
             * @param data double pointer to data which will be adusted
             *
             * @return None
             */
            void mutate(geometry_msgs::Point* data) override;

        private:

            /**@private
             * @brief position2d mutator that does the mutation
             */
            Position2D::SwapXYMutator m_swapMutator;
    };



    /**
     * @brief Mutator that puts a value into Z in a given Point
     */
    class StuckZMutator: public PointMsgMutator
    {

        public:

            /**
             * @brief creates a mutator that will set given Pointmsg's Z
             *
             * @param setZValue double to set Z to for each Point
             */
            StuckZMutator(double setZValue);

            /**
             * @brief takes a pointer to a Pointmsg and sets the Z
             *
             * @param data double pointer to data which will be adusted
             *
             * @return None
             */
            void mutate(geometry_msgs::Point* data) override;

        private:

            /**
             * @private
             *
             * @brief float mutator that does the mutation
             */
            Double::StuckValueMutator m_stuckMutator;
    };




}; // end PointMsg namespace

#endif