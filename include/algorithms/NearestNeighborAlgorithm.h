/*
 * NearestNeighborAlgorithm.h
 *
 *  Created on: Mar 8, 2015
 *      Author: andre
 */

#ifndef RS_LOG_LEARN_SRC_ALGORITHMS_NEARESTNEIGHBORALGORITHM_H_
#define RS_LOG_LEARN_SRC_ALGORITHMS_NEARESTNEIGHBORALGORITHM_H_

#include <algorithms/mpAlgorithm.h>
#include <iai_rs/util/time.h>

namespace rs_log_learn
{

class NearestNeighborAlgorithm: public mpAlgorithm
{
public:
    NearestNeighborAlgorithm();
    virtual ~NearestNeighborAlgorithm();
    void process(std::vector<MPIdentifiable> referenceSet, MPIdentifiable query);
};

} /* namespace rs_log_learn */

#endif /* RS_LOG_LEARN_SRC_ALGORITHMS_NEARESTNEIGHBORALGORITHM_H_ */
