/*
 * GroundTruth.h
 *
 *  Created on: Feb 25, 2015
 *      Author: andre
 */

#ifndef RS_LOG_LEARN_SRC_GROUNDTRUTH_H_
#define RS_LOG_LEARN_SRC_GROUNDTRUTH_H_

#include <string>

namespace rs_log_learning
{

class GroundTruth
{
public:
    GroundTruth(std::string globalGt);
    GroundTruth() {};
    virtual ~GroundTruth();

    const std::string& getGlobaltGt() const { return globaltGt_; }
    const std::string& getShape() const { return shape_; }

    void setGlobaltGt(const std::string& globaltGt) { globaltGt_ = globaltGt; }
    void setShape(const std::string& shape) { shape_ = shape; }

private:
    std::string globaltGt_;
    std::string shape_;
};

} /* namespace rs_log_learning */

#endif /* RS_LOG_LEARN_SRC_GROUNDTRUTH_H_ */
