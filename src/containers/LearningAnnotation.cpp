/*
 * LearningAnnotation.cpp
 *
 *  Created on: Feb 18, 2015
 *      Author: andre
 */

#include <containers/LearningAnnotation.h>

namespace rs_log_learn
{

LearningAnnotation::LearningAnnotation(std::string learnedObject)
{
    this->learnedObjectName_ = learnedObject;
}

LearningAnnotation::~LearningAnnotation()
{
}

} /* namespace rs_log_learn */
