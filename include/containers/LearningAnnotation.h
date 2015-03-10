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

    std::string& getLearnedName(){ return learnedObjectName_; }
    std::string& getShape() { return shape_; }
    float getConfidence() { return confidence_; }

    void setLearnedName(const std::string& learnedObject) { this->learnedObjectName_ = learnedObject; }
    void setShape(const std::string& shape) { this->shape_ = shape; }
    void setConfidence(float confidence) { this->confidence_ = confidence; }

private:
    std::string learnedObjectName_;
    std::string shape_;
    float confidence_;
};

} /* namespace rs_log_learn */

#endif /* RS_LOG_LEARN_SRC_LEARNINGANNOTATION_H_ */
