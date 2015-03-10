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
        outInfo("MPident GT:  " << it->getGroundTruth().getGlobaltGt() << "  gt shape: " << it->getGroundTruth().getShape());
    }

    arma::mat queryData(1, 1, arma::fill::zeros);
    arma::mat referenceData(1, 1, arma::fill::zeros);

    queryData(0,0) = query.getGeometry().getBoundingBoxVolume();
    for(int i = 1; i <= referenceSet.size(); ++i)
    {
        referenceData.resize(1,i);
        referenceData(0,i-1) = referenceSet[i].getGeometry().getBoundingBoxVolume();
    }
    std::cout << "set " << i-1 << "reference objects" << std::endl;
    std::cout << "before knn search" << std::endl;
    AllkNN a(referenceData, queryData);
    std::cout << "after knn search" << std::endl;

    std::cout << "input:" << std::endl;
    referenceData.print();
    std::cout << "------------------" << std::endl;
    std::cout << "query:" << std::endl;
    queryData.print();

    arma::Mat<size_t> resultingNeighbors;
    arma::mat resultingDistances;

    a.Search(1, resultingNeighbors, resultingDistances);

    // Write each neighbor and distance using Log.
    for (size_t i = 0; i < resultingNeighbors.n_elem; ++i)
    {
        std::cout << "Nearest neighbor of point " << i << " is point "
                << resultingNeighbors[i] << " and the distance is " << resultingDistances[i] << std::endl;
        std::cout << "GT of matched reference is: " <<
                referenceSet[resultingNeighbors[i]].getGroundTruth().getGlobaltGt() << std::endl;
    }

    outInfo("took: " << clock.getTime() << " ms.");
}

} /* namespace rs_log_learn */
