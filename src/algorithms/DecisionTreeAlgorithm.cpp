/*
 * DecisionTreeAlgorithm.cpp
 *
 *  Created on: Mar 17, 2015
 *      Author: andre
 */

#include "algorithms/DecisionTreeAlgorithm.h"

namespace rs_log_learn
{

DecisionTreeAlgorithm::DecisionTreeAlgorithm()
{

}

DecisionTreeAlgorithm::~DecisionTreeAlgorithm()
{
}

MPIdentifiable DecisionTreeAlgorithm::process(std::vector<MPIdentifiable> referenceSet, MPIdentifiable query)
{
    iai_rs::util::StopWatch clock;

    cv::Mat trainingData(10, 10, CV_32FC1);
    cv::Mat testData(10, 10, CV_32FC1);

    for(int i = 0; i < referenceSet.size(); ++i)
    {

    }

    cv::randu(trainingData,0,10);
    cv::randu(testData,0,10);

    std::cout << "TrainingData: " << trainingData << std::endl;
    std::cout << "testData: " << testData << std::endl;

    int eq = 0;

    cv::Mat trainingClasses = labelData(trainingData, eq);
    cv::Mat testClasses = labelData(testData, eq);

    decisiontree(trainingData, trainingClasses, testData, testClasses);


    outInfo("took: " << clock.getTime() << " ms.");
    outInfo("learned name for query: " << query.getLearningAnnotation().getLearnedName());
    return query;
}

cv::Mat DecisionTreeAlgorithm::labelData(cv::Mat points, int equation)
{
    cv::Mat labels(points.rows, 1, CV_32FC1);
    for(int i = 0; i < points.rows; i++) {
             float x = points.at<float>(i,0);
             float y = points.at<float>(i,1);
             labels.at<float>(i, 0) = f(x, y, equation);
        }
    return labels;
}

void DecisionTreeAlgorithm::decisiontree(cv::Mat& trainingData, cv::Mat& trainingClasses, cv::Mat& testData, cv::Mat& testClasses)
{
    CvDTree dtree;

    uint8_t *data = new uint8_t[11]; // initial row data for type mask
    for(int i = 0; i < 11; ++i)
    {
        data[i] = 0;
    }

    cv::Mat var_type(11, 1, CV_8UC1, data);

    std::cout << "Mat: " << var_type << std::endl;

    outInfo("starting dtree");


    dtree.train(trainingData,CV_ROW_SAMPLE, trainingClasses, cv::Mat(), cv::Mat(), var_type, cv::Mat(),
            CvDTreeParams(8,    // max depth
                          5,    // min sample count
                          0,    // regression N/A
                          true, // compute surrogate split for missing data
                          15,   // max number of categories
                          1,    // number of cross validation folds
                          true, // 1SE rule -> smaller tree
                          true, // throw away pruned tree branches
                          0));  // no priors
    cv::Mat predicted(testClasses.rows, 1, CV_32F);
    for (int i = 0; i < testData.rows; i++) {
        const cv::Mat sample = testData.row(i);
        CvDTreeNode* prediction = dtree.predict(sample);
        predicted.at<float> (i, 0) = prediction->value;
    }

    std::cout << "testClasses = " << testClasses << std::endl;
    std::cout << "predicted = " << predicted << std::endl;
    std::cout << "Accuracy_{TREE} = " << evaluate(predicted, testClasses) << std::endl;
    std::cout << "Var importance mat = " << dtree.getVarImportance() << std::endl;

    delete[] data;
}

int DecisionTreeAlgorithm::f(float x, float y, int equation)
{
    if(x < 1) return 1;
    if(x < 2 && x > 1) return 2;
    if(x < 3 && x > 2) return 3;
    if(x < 4 && x > 3) return 4;
    if(x < 5 && x > 4) return 5;
    if(x < 6 && x > 5) return 6;
    if(x < 7 && x > 6) return 7;
    if(x < 8 && x > 7) return 8;
    if(x < 9 && x > 8) return 9;
    if(x < 10 && x > 9) return 10;
    else return 0;

}

float DecisionTreeAlgorithm::evaluate(cv::Mat& predicted, cv::Mat& actual)
{
    assert(predicted.rows == actual.rows);
    int t = 0;
    int f = 0;
    for(int i = 0; i < actual.rows; i++) {
        float p = predicted.at<float>(i,0);
        float a = actual.at<float>(i,0);
        std::cout << "evaluate: predicted/actual: " << p << " / " << a << "  " << (int)round(p) << "/" << (int)round(a) << std::endl;
        if((int)(round(p) == (int)(round(a))))
        {
            t++;
        }
        else {
            f++;
        }
    }
    return (t * 1.0) / (t + f);
}

} /* namespace rs_log_learn */
