/*
 * mpAlgorithm.h
 *
 *  Created on: Mar 8, 2015
 *      Author: andre
 */

#ifndef RS_LOG_LEARN_SRC_ALGORITHMS_MPALGORITHM_H_
#define RS_LOG_LEARN_SRC_ALGORITHMS_MPALGORITHM_H_

#include <rs_log_learning/MPIdentifiable.h>

namespace rs_log_learning
{

class mpAlgorithm
{
public:
  virtual ~mpAlgorithm();
  virtual MPIdentifiable process(std::vector<MPIdentifiable> referenceSet, MPIdentifiable query) = 0;
};

} /* namespace rs_log_learning */

#endif /* RS_LOG_LEARN_SRC_ALGORITHMS_MPALGORITHM_H_ */
