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

MPIdentifiable NearestNeighborAlgorithm::process(std::vector<MPIdentifiable> referenceSet, MPIdentifiable query)
{
    iai_rs::util::StopWatch clock;

    outInfo("Query Ident size:  " << query.getGeometry().getSize());
    outInfo("Query color  " << query.getSemColor().getColorMapping()["white"]);
    outInfo("Query color  " << query.getSemColor().getColorMapping()["yellow"]);
    outInfo("Query color  " << query.getSemColor().getColorMapping()["red"]);
    outInfo("Query color  " << query.getSemColor().getColorMapping()["black"]);
    outInfo("Query color  " << query.getSemColor().getColorMapping()["grey"]);
    outInfo("Query color  " << query.getSemColor().getColorMapping()["blue"]);
    outInfo("Query color  " << query.getSemColor().getColorMapping()["green"]);
    outInfo("Query color  " << query.getSemColor().getColorMapping()["magenta"]);
    outInfo("Query color  " << query.getSemColor().getColorMapping()["cyan"]);

    for(std::vector<MPIdentifiable>::iterator it = referenceSet.begin(); it != referenceSet.end(); ++it)
    {
        if(!it->getSemColor().getColorMapping().empty())
        {
            std::map<std::string, float> semColorMap = it->getSemColor().getColorMapping();
            float whiteRatio = semColorMap["white"];
            outInfo("MPident white value:  " << whiteRatio);
        }
        else
        {
            outError("No SemColorMap in this reference identifiable");
        }
        outInfo("MPident GT:  " << it->getGroundTruth().getGlobaltGt() << "  gt shape: " << it->getGroundTruth().getShape());
    }

    arma::mat queryData(10, 1, arma::fill::zeros);
    arma::mat referenceData(10, 1, arma::fill::zeros);

    queryData(0,0) = query.getGeometry().getBoundingBoxVolume();
    queryData(1,0) = query.getSemColor().getColorMapping()["white"];
    queryData(2,0) = query.getSemColor().getColorMapping()["yellow"];
    queryData(3,0) = query.getSemColor().getColorMapping()["red"];
    queryData(4,0) = query.getSemColor().getColorMapping()["black"];
    queryData(5,0) = query.getSemColor().getColorMapping()["grey"];
    queryData(6,0) = query.getSemColor().getColorMapping()["blue"];
    queryData(7,0) = query.getSemColor().getColorMapping()["green"];
    queryData(8,0) = query.getSemColor().getColorMapping()["magenta"];
    queryData(9,0) = query.getSemColor().getColorMapping()["cyan"];

    // add geometry and color data to referenceData matrix
    int i = 0;
    for(i = 0; i < referenceSet.size(); ++i)
    {
        referenceData.resize(10,i+1);
        referenceData(0,i) = referenceSet[i].getGeometry().getBoundingBoxVolume();
        referenceData(1,i) = referenceSet[i].getSemColor().getColorMapping()["white"];
        referenceData(2,i) = referenceSet[i].getSemColor().getColorMapping()["yellow"];
        referenceData(3,i) = referenceSet[i].getSemColor().getColorMapping()["red"];
        referenceData(4,i) = referenceSet[i].getSemColor().getColorMapping()["black"];
        referenceData(5,i) = referenceSet[i].getSemColor().getColorMapping()["grez"];
        referenceData(6,i) = referenceSet[i].getSemColor().getColorMapping()["blue"];
        referenceData(7,i) = referenceSet[i].getSemColor().getColorMapping()["green"];
        referenceData(8,i) = referenceSet[i].getSemColor().getColorMapping()["magenta"];
        referenceData(9,i) = referenceSet[i].getSemColor().getColorMapping()["cyan"];
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
        LearningAnnotation lrn;
        lrn.setLearnedName(referenceSet[resultingNeighbors[i]].getGroundTruth().getGlobaltGt());
        lrn.setShape(referenceSet[resultingNeighbors[i]].getGroundTruth().getShape());
        query.setLearningAnnotation(lrn);
    }

    outInfo("took: " << clock.getTime() << " ms.");
    outInfo("query lrn: " << query.getLearningAnnotation().getLearnedName());
    return query;
}

} /* namespace rs_log_learn */
