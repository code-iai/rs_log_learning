/*
 * GroundTruth.cpp
 *
 *  Created on: Feb 25, 2015
 *      Author: andre
 */

#include "containers/GroundTruth.h"

namespace rs_log_learn
{

GroundTruth::GroundTruth(std::string globalGt)
{
    globaltGt_ = globalGt;
}

GroundTruth::~GroundTruth()
{
}

} /* namespace rs_log_learn */
