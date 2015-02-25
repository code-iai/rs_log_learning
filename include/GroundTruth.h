/*
 * GroundTruth.h
 *
 *  Created on: Feb 25, 2015
 *      Author: andre
 */

#ifndef RS_LOG_LEARN_SRC_GROUNDTRUTH_H_
#define RS_LOG_LEARN_SRC_GROUNDTRUTH_H_

namespace rs_log_learn
{

class GroundTruth
{
public:
    GroundTruth();
    virtual ~GroundTruth();

private:
    std::string globatGt;
    std::string shape;
};

} /* namespace rs_log_learn */

#endif /* RS_LOG_LEARN_SRC_GROUNDTRUTH_H_ */
