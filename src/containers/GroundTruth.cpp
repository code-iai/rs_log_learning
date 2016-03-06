/*
 * GroundTruth.cpp
 *
 *  Created on: Feb 25, 2015
 *      Author: andre
 */

#include <rs_log_learning/containers/GroundTruth.h>

namespace rs_log_learning
{

GroundTruth::GroundTruth(std::string globalGt)
{
    globaltGt_ = globalGt;
}

GroundTruth::~GroundTruth()
{
}

} /* namespace rs_log_learning */
