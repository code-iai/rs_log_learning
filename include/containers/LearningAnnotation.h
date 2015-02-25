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
    LearningAnnotation()
    {
    }
    ;
    virtual ~LearningAnnotation();

    std::string& getLearnedName(){ return learnedObjectName_; }
    std::string& getShape() { return shape; }

    void setLearnedName(const std::string& learnedObject) { this->learnedObjectName_ = learnedObject; }
    void setShape(const std::string& shape) { this->shape = shape; }

private:
    std::string learnedObjectName_;
    std::string shape;
};

} /* namespace rs_log_learn */

#endif /* RS_LOG_LEARN_SRC_LEARNINGANNOTATION_H_ */
