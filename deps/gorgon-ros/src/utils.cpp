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

#include <gorgon_ros/utils.h>

/// Some macros so we can add these things to the mutator map more easily
#define ADD_ReplaceWholeArray(primitiveType, primitiveTypeString) \
    m_MutatorMap[std::string(primitiveTypeString) + "_ReplaceWholeArrayMutator"] = \
    [](json args, unsigned long seed) -> std::shared_ptr<RosBaseMutator> { \
        return std::make_shared<ArraySerialized::ReplaceWholeArrayMutator<primitiveType>>(args.at("replacementVal").get<primitiveType>(), \
                                                                                          args.at("arraySize").get<std::size_t>()); \
    };

#define ADD_ProbabalisticReplace(primitiveType, primitiveTypeString) \
    m_MutatorMap[std::string(primitiveTypeString) + "_ProbabalisticReplaceMutator"] = \
    [](json args, unsigned long seed) -> std::shared_ptr<RosBaseMutator> { \
        return std::make_shared<ArraySerialized::ProbabalisticReplaceMutator<primitiveType>>(args.at("replacementValue").get<primitiveType>(), \
                                                                                            args.at("probability").get<float>(), \
                                                                                            args.at("arraySize").get<std::size_t>(), \
                                                                                            seed); \
    };    

#define ADD_AddToWholeArray(primitiveType, primitiveTypeString) \
    m_MutatorMap[std::string(primitiveTypeString) + "_AddToWholeArrayMutator"] = \
    [](json args, unsigned long seed) -> std::shared_ptr<RosBaseMutator> { \
        return std::make_shared<ArraySerialized::AddToWholeArrayMutator<primitiveType>>(args.at("valueToAdd").get<primitiveType>(), \
                                                                                       args.at("minArrayValue").get<primitiveType>(), \
                                                                                       args.at("maxArrayValue").get<primitiveType>(), \
                                                                                       args.at("arraySize").get<std::size_t>()); \
    };

#define ADD_FOR_ALL_TYPES(mutatorType) \
    ADD_##mutatorType(double, "Float64Array") \
    ADD_##mutatorType(float, "Float32Array") \
    ADD_##mutatorType(int, "Int32Array") \
    ADD_##mutatorType(unsigned int, "UInt32Array") \


// Begin RosWrapperMutator
utils::RosWrapperMutator::RosWrapperMutator(std::shared_ptr<BaseMutator> wrappedMutator)
    : m_wrappedMutator(wrappedMutator)
{ }

void utils::RosWrapperMutator::mutateSerialized(void* data, size_t data_size)
{
    m_wrappedMutator->mutateRef(data);
}

void utils::RosWrapperMutator::mutateRef(void* data)
{
    m_wrappedMutator->mutateRef(data);
}

void utils::RosWrapperMutator::mutateByReferenceSeeded(void* data, unsigned long seed)
{
    m_wrappedMutator->mutateByReferenceSeeded(data, seed);
}

void utils::RosWrapperMutator::mutateSerializedSeeded(void* data, size_t dataSize, unsigned long seed)
{
    m_wrappedMutator->mutateByReferenceSeeded(data, seed);
}
// End RosWrapperMutator

// begin RosMutatorFactory
utils::RosMutatorFactory::RosMutatorFactory()
    : m_gorgonDefaultFactory(MutatorFactory())
{
    // set Mutator Creation Lambda
    m_MutatorMap["PointMsg_AnisotropicGaussianNoiseMutator"] =
    [](json args, unsigned long seed) -> std::shared_ptr<RosBaseMutator> {
        return std::make_shared<PointMsg::AnisotropicGaussianNoiseMutator>(args.at("meanX"),
                args.at("stdDevX").get<double>(),
                args.at("meanY").get<double>(),
                args.at("stdDevY").get<double>(),
                seed
                                                                        );
    };

    m_MutatorMap["PointMsg_SwapXYMutator"] =
    [](json args, unsigned long seed) -> std::shared_ptr<RosBaseMutator> {
        return std::make_shared<PointMsg::SwapXYMutator>();
    };

    m_MutatorMap["PointMsg_StuckZMutator"] =
    [](json args, unsigned long seed) -> std::shared_ptr<RosBaseMutator> {
        return std::make_shared<PointMsg::StuckZMutator>(args.at("stuckZVal").get<double>()
                                                        );
    };

    ADD_FOR_ALL_TYPES(ReplaceWholeArray)
    ADD_FOR_ALL_TYPES(ProbabalisticReplace)
    ADD_FOR_ALL_TYPES(AddToWholeArray)
}


std::shared_ptr<RosBaseMutator> utils::RosMutatorFactory::buildMutator(std::string mutatorType, json args, unsigned long seed)
{
    auto search_result = m_MutatorMap.find(mutatorType);

    if (search_result != m_MutatorMap.end()) {
        return search_result->second(args, seed);
    } else {
        // if it isn't in our list, try to find it in default gorgon and wrap it for ros usage
        return std::make_shared<utils::RosWrapperMutator>(
                   m_gorgonDefaultFactory.buildMutator(mutatorType, args, seed)
               );
    }

}

std::shared_ptr<RosBaseMutator> utils::RosMutatorFactory::buildMutator(std::string mutatorType, json args)
{
    return buildMutator(mutatorType, args, std::random_device()());
}
// End RosMutatorFactory