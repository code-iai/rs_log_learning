/*
 * LearningAnnotation.cpp
 *
 *  Created on: Feb 18, 2015
 *      Author: andre
 */

#include "LearningAnnotation.h"

namespace rs_log_learn
{

LearningAnnotation::LearningAnnotation(std::string learnedObject)
{
	this->learnedObject_ = learnedObject;
}

LearningAnnotation::~LearningAnnotation()
{
}

} /* namespace rs_log_learn */
