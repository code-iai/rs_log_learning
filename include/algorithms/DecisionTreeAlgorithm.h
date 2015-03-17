/*
 * DecisionTreeAlgorithm.h
 *
 *  Created on: Mar 17, 2015
 *      Author: andre
 */

#ifndef RS_LOG_LEARN_SRC_ALGORITHMS_DECISIONTREEALGORITHM_H_
#define RS_LOG_LEARN_SRC_ALGORITHMS_DECISIONTREEALGORITHM_H_

#include <algorithms/mpAlgorithm.h>
#include "cv.h"
#include "ml.h"

namespace rs_log_learn
{

class DecisionTreeAlgorithm: public mpAlgorithm
{
public:
    DecisionTreeAlgorithm();
    virtual ~DecisionTreeAlgorithm();

    MPIdentifiable process(std::vector<MPIdentifiable> referenceSet, MPIdentifiable query);
    cv::Mat labelData(cv::Mat points, int equation);

    void decisiontree(cv::Mat& trainingData, cv::Mat& trainingClasses, cv::Mat& testData, cv::Mat& testClasses);
    int f(float x, float y, int equation);
    float evaluate(cv::Mat& predicted, cv::Mat& actual);
};

} /* namespace rs_log_learn */

#endif /* RS_LOG_LEARN_SRC_ALGORITHMS_DECISIONTREEALGORITHM_H_ */
