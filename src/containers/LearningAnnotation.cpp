/*
 * LearningAnnotation.cpp
 *
 *  Created on: Feb 18, 2015
 *      Author: andre
 */

#include <rs_log_learning/containers/LearningAnnotation.h>

namespace rs_log_learning
{

LearningAnnotation::LearningAnnotation(std::string learnedObject)
{
  this->learnedObjectName_ = learnedObject;
}

LearningAnnotation::~LearningAnnotation()
{
}

} /* namespace rs_log_learning */
