/*
 * LearningAnnotation.h
 *
 *  Created on: Feb 18, 2015
 *      Author: andre
 */

#ifndef RS_LOG_LEARN_SRC_LEARNINGANNOTATION_H_
#define RS_LOG_LEARN_SRC_LEARNINGANNOTATION_H_

#include <iostream>

namespace rs_log_learn
{

class LearningAnnotation
{
public:
	LearningAnnotation(std::string learnedObject);
	LearningAnnotation() {};
	virtual ~LearningAnnotation();

	std::string& getLearnedObject() { return learnedObject_; }
	void setLearnedObject(const std::string& learnedObject) { this->learnedObject_ = learnedObject; }

private:
	std::string learnedObject_;
};

} /* namespace rs_log_learn */

#endif /* RS_LOG_LEARN_SRC_LEARNINGANNOTATION_H_ */
