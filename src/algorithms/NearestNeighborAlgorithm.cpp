/*
 * NearestNeighborAlgorithm.cpp
 *
 *  Created on: Mar 8, 2015
 *      Author: andre
 */

#include "algorithms/NearestNeighborAlgorithm.h"

namespace rs_log_learn
{

NearestNeighborAlgorithm::NearestNeighborAlgorithm()
{
}

NearestNeighborAlgorithm::~NearestNeighborAlgorithm()
{
}

void NearestNeighborAlgorithm::process(std::vector<MPIdentifiable> referenceSet, MPIdentifiable query)
{
    iai_rs::util::StopWatch clock;

    outInfo("Query Ident size:  " << query.getGeometry().getSize());
    for(std::vector<MPIdentifiable>::iterator it = referenceSet.begin(); it != referenceSet.end(); ++it)
    {
        outInfo("MPident geom size:  " << it->getGeometry().getSize());
    }

    outInfo("took: " << clock.getTime() << " ms.");
}

} /* namespace rs_log_learn */
