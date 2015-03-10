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

#include <mlpack/core.hpp>
#include <mlpack/methods/neighbor_search/neighbor_search.hpp>

namespace rs_log_learn
{

using namespace mlpack;
using namespace mlpack::neighbor; // NeighborSearch and NearestNeighborSort

class NearestNeighborAlgorithm: public mpAlgorithm
{
public:
    NearestNeighborAlgorithm();
    virtual ~NearestNeighborAlgorithm();
    MPIdentifiable process(std::vector<MPIdentifiable> referenceSet, MPIdentifiable query);

private:
    float computeVectorLength(arma::mat matrix, size_t i);
};

} /* namespace rs_log_learn */

#endif /* RS_LOG_LEARN_SRC_ALGORITHMS_NEARESTNEIGHBORALGORITHM_H_ */
