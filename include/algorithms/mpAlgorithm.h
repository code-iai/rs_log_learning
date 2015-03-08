/*
 * mpAlgorithm.h
 *
 *  Created on: Mar 8, 2015
 *      Author: andre
 */

#ifndef RS_LOG_LEARN_SRC_ALGORITHMS_MPALGORITHM_H_
#define RS_LOG_LEARN_SRC_ALGORITHMS_MPALGORITHM_H_

#include <MPIdentifiable.h>

namespace rs_log_learn
{

class mpAlgorithm
{
public:
    virtual ~mpAlgorithm();
    virtual void process(std::vector<MPIdentifiable> referenceSet, MPIdentifiable query) = 0;
};

} /* namespace rs_log_learn */

#endif /* RS_LOG_LEARN_SRC_ALGORITHMS_MPALGORITHM_H_ */
