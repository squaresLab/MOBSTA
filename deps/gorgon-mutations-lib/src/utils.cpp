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

#include <gorgon/utils.h>

/// Some macros so we can add these things to the mutator map more easily
#define ADD_ReplaceWholeArray(primitiveType, primitiveTypeString) \
    m_MutatorMap[std::string(primitiveTypeString) + "_ReplaceWholeArrayMutator"] = \
    [](json args, unsigned long seed) -> std::shared_ptr<BaseMutator> { \
        return std::make_shared<FixedSizeNumericalArray::ReplaceWholeArrayMutator<primitiveType>>(args.at("replacementVal").get<primitiveType>(), \
                                                                                                  args.at("arraySize").get<std::size_t>()); \
    };

#define ADD_ProbabalisticReplace(primitiveType, primitiveTypeString) \
    m_MutatorMap[std::string(primitiveTypeString) + "_ProbabalisticReplaceMutator"] = \
    [](json args, unsigned long seed) -> std::shared_ptr<BaseMutator> { \
        return std::make_shared<FixedSizeNumericalArray::ProbabalisticReplaceMutator<primitiveType>>(args.at("replacementValue").get<primitiveType>(), \
                                                                                                     args.at("probability").get<float>(), \
                                                                                                     args.at("arraySize").get<std::size_t>(), \
                                                                                                     seed); \
    };    

#define ADD_AddToWholeArray(primitiveType, primitiveTypeString) \
    m_MutatorMap[std::string(primitiveTypeString) + "_AddToWholeArrayMutator"] = \
    [](json args, unsigned long seed) -> std::shared_ptr<BaseMutator> { \
        return std::make_shared<FixedSizeNumericalArray::AddToWholeArrayMutator<primitiveType>>(args.at("valueToAdd").get<primitiveType>(), \
                                                                                                args.at("minArrayValue").get<primitiveType>(), \
                                                                                                args.at("maxArrayValue").get<primitiveType>(), \
                                                                                                args.at("arraySize").get<std::size_t>()); \
    };

#define ADD_FOR_ALL_TYPES(mutatorType) \
    ADD_##mutatorType(double, "Float64Array") \
    ADD_##mutatorType(float, "Float32Array") \
    ADD_##mutatorType(int, "Int32Array") \
    ADD_##mutatorType(unsigned int, "UInt32Array") \



/// Begin MutatorFactory

utils::MutatorFactory::MutatorFactory()
{
    // set Mutator Creation Lambda
    m_MutatorMap["Double_GaussianNoiseMutator"] =
    [](json args, unsigned long seed) -> std::shared_ptr<BaseMutator> {

        return std::make_shared<Double::GaussianNoiseMutator>(args.at("mean").get<double>(),
                                                              args.at("stdDev").get<double>(),
                                                              seed
                                                             );
    };

    m_MutatorMap["Double_DictionaryAttackMutator"] =
    [](json args, unsigned long seed) -> std::shared_ptr<BaseMutator> {
        return std::make_shared<Double::DictionaryAttackMutator>(seed
        );
    };

    // first arg must be char
    // json .get does json->uint->char, this goes json->string->first element so it doesnt need to be ascii code
    m_MutatorMap["CharArray_ReplaceCharMutator"] =
    [](json args, unsigned long seed) -> std::shared_ptr<BaseMutator> {
        return std::make_shared<CharArray::ReplaceCharMutator>( (args.at("replacement").get<std::string>().at(0)),
                                                                 args.at("index").get<int>()
        );
    };
    
    m_MutatorMap["Double_StuckValueMutator"] =
    [](json args, unsigned long seed) -> std::shared_ptr<BaseMutator> {
        return std::make_shared<Double::StuckValueMutator>(args.at("stuckValue").get<double>()
                                                  );
    };

    m_MutatorMap["Position2D_AnisotropicGaussianNoiseMutator"] =
    [](json args, unsigned long seed) -> std::shared_ptr<BaseMutator> {
        return std::make_shared<Position2D::AnisotropicGaussianNoiseMutator>(args.at("meanX").get<double>(),
                args.at("stdDevX").get<double>(),
                args.at("meanY").get<double>(),
                args.at("stdDevY").get<double>(),
                seed
        );
    };

    m_MutatorMap["Position2D_SwapXYMutator"] =
    [](json args, unsigned long seed) -> std::shared_ptr<BaseMutator> {
        return std::make_shared<Position2D::SwapXYMutator>(
        );
    };


    ADD_FOR_ALL_TYPES(ReplaceWholeArray)
    ADD_FOR_ALL_TYPES(ProbabalisticReplace)
    ADD_FOR_ALL_TYPES(AddToWholeArray)
}


std::shared_ptr<BaseMutator> utils::MutatorFactory::buildMutator(std::string mutatorType, json args, unsigned long seed)
{

    auto search_result = m_MutatorMap.find(mutatorType);

    if (search_result != m_MutatorMap.end()) {
        return search_result->second(args, seed);
    } else {
        throw std::invalid_argument(mutatorType + " blueprint not found in Mutator Factory");
    }

}

std::shared_ptr<BaseMutator> utils::MutatorFactory::buildMutator(std::string mutatorType, json args)
{
    return buildMutator(mutatorType, args, std::random_device()());
}
