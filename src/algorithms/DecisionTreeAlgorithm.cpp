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
    mm = 10;
}

DecisionTreeAlgorithm::~DecisionTreeAlgorithm()
{
}

MPIdentifiable DecisionTreeAlgorithm::process(std::vector<MPIdentifiable> referenceSet, MPIdentifiable query)
{
    iai_rs::util::StopWatch clock;
    outInfo("========================== dt start ====================");

    float *trainingData = new float[10*referenceSet.size()]; // data for training
    float *testData = new float[10]; // query data

    for(int i = 0; i < referenceSet.size(); ++i)
    {
        trainingData[i*10] = (float)referenceSet[i].getGeometry().getBoundingBoxVolume();
        if(!referenceSet[i].getSemColor().getColorMapping().empty())
        {
            trainingData[i*10 + 1] = referenceSet[i].getSemColor().getColorMapping()["white"];
            trainingData[i*10 + 2] = referenceSet[i].getSemColor().getColorMapping()["yellow"];
            trainingData[i*10 + 3] = referenceSet[i].getSemColor().getColorMapping()["red"];
            trainingData[i*10 + 4] = referenceSet[i].getSemColor().getColorMapping()["black"];
            trainingData[i*10 + 5] = referenceSet[i].getSemColor().getColorMapping()["grey"];
            trainingData[i*10 + 6] = referenceSet[i].getSemColor().getColorMapping()["blue"];
            trainingData[i*10 + 7] = referenceSet[i].getSemColor().getColorMapping()["green"];
            trainingData[i*10 + 8] = referenceSet[i].getSemColor().getColorMapping()["magenta"];
            trainingData[i*10 + 9] = referenceSet[i].getSemColor().getColorMapping()["cyan"];
        }
        else
        {
            outError("No SemColorMap in this reference identifiable");
        }
    }

    testData[0] = (float)query.getGeometry().getBoundingBoxVolume();
    if(!query.getSemColor().getColorMapping().empty())
    {
        testData[1] = query.getSemColor().getColorMapping()["white"];
        testData[2] = query.getSemColor().getColorMapping()["yellow"];
        testData[3] = query.getSemColor().getColorMapping()["red"];
        testData[4] = query.getSemColor().getColorMapping()["black"];
        testData[5] = query.getSemColor().getColorMapping()["grey"];
        testData[6] = query.getSemColor().getColorMapping()["blue"];
        testData[7] = query.getSemColor().getColorMapping()["green"];
        testData[8] = query.getSemColor().getColorMapping()["magenta"];
        testData[9] = query.getSemColor().getColorMapping()["cyan"];
    }
    else
    {
        outError("No SemColorMap in this reference identifiable");
    }

    float *X10trainingData = new float[10*10*referenceSet.size()]; // data for training
    std::copy(trainingData, trainingData + 10*referenceSet.size(), X10trainingData);
    std::copy(trainingData, trainingData + 10*referenceSet.size(), X10trainingData + 1*10*referenceSet.size());
    std::copy(trainingData, trainingData + 10*referenceSet.size(), X10trainingData + 2*10*referenceSet.size());
    std::copy(trainingData, trainingData + 10*referenceSet.size(), X10trainingData + 3*10*referenceSet.size());
    std::copy(trainingData, trainingData + 10*referenceSet.size(), X10trainingData + 4*10*referenceSet.size());
    std::copy(trainingData, trainingData + 10*referenceSet.size(), X10trainingData + 5*10*referenceSet.size());
    std::copy(trainingData, trainingData + 10*referenceSet.size(), X10trainingData + 6*10*referenceSet.size());
    std::copy(trainingData, trainingData + 10*referenceSet.size(), X10trainingData + 7*10*referenceSet.size());
    std::copy(trainingData, trainingData + 10*referenceSet.size(), X10trainingData + 8*10*referenceSet.size());
    std::copy(trainingData, trainingData + 10*referenceSet.size(), X10trainingData + 9*10*referenceSet.size());

    cv::Mat testMat(1, 10, CV_32FC1, testData);
    cv::Mat trainingMat(10*referenceSet.size(), 10, CV_32FC1, X10trainingData);

    std::cout << "TrainingData: " << trainingMat << std::endl;
    std::cout << "testData: " << testMat << std::endl;

    outInfo("before trainingClasses");
    cv::Mat trainingClasses = labelData(referenceSet);
    cv::Mat testClasses = labelData(referenceSet); // TODO: not needed, get just the prediction

    std::cout << "trainingClasses: " << trainingClasses << std::endl;

    float resultValue = decisiontree(trainingMat, trainingClasses, testMat, testClasses);
    float confidence = 1 - abs(resultValue - (size_t)round(resultValue));
    size_t resultIndex = (size_t)round(resultValue);

    LearningAnnotation lrn;
    lrn.setLearnedName(referenceSet[resultIndex].getGroundTruth().getGlobaltGt());
    lrn.setShape(referenceSet[resultIndex].getGroundTruth().getShape());
    lrn.setConfidence(confidence); // TODO: get value

    query.setLearningAnnotation(lrn);

    outInfo("took: " << clock.getTime() << " ms.");
    outInfo("learned name for query: " << query.getLearningAnnotation().getLearnedName());

    delete[] trainingData;
    delete[] testData;

    return query;
}

/*
 * construct the classification labels from the reference set
 * matching the labels from the identifiables to a set of numeric values for dtree processing
 */
cv::Mat DecisionTreeAlgorithm::labelData(std::vector<MPIdentifiable> &referenceSet)
{
    outInfo("labeling referenceData");
    // group the same ground truth data in the map
    for(int i = 0; i < referenceSet.size(); ++i)
    {
        NameShape ns;
        if(referenceSet[i].getGroundTruth().getGlobaltGt().empty())
        {
            outError("no GT available");
            // add learned data instead
        }
        ns.name = referenceSet[i].getGroundTruth().getGlobaltGt();
        ns.shape = referenceSet[i].getGroundTruth().getShape();
        indexToAnnotationData[i] = ns;
    }
    outInfo("classification map size: " << indexToAnnotationData.size());

    int labelIndex = 0;
    cv::Mat labels(indexToAnnotationData.size()*10, 1, CV_32FC1);
    for(std::map<int,NameShape>::iterator it = indexToAnnotationData.begin(); it != indexToAnnotationData.end(); ++it)
    {
        labels.at<float>(labelIndex, 0) = (float)it->first;
        labels.at<float>(labelIndex + indexToAnnotationData.size() * 1, 0) = (float)it->first;
        labels.at<float>(labelIndex + indexToAnnotationData.size() * 2, 0) = (float)it->first;
        labels.at<float>(labelIndex + indexToAnnotationData.size() * 3, 0) = (float)it->first;
        labels.at<float>(labelIndex + indexToAnnotationData.size() * 4, 0) = (float)it->first;
        labels.at<float>(labelIndex + indexToAnnotationData.size() * 5, 0) = (float)it->first;
        labels.at<float>(labelIndex + indexToAnnotationData.size() * 6, 0) = (float)it->first;
        labels.at<float>(labelIndex + indexToAnnotationData.size() * 7, 0) = (float)it->first;
        labels.at<float>(labelIndex + indexToAnnotationData.size() * 8, 0) = (float)it->first;
        labels.at<float>(labelIndex + indexToAnnotationData.size() * 9, 0) = (float)it->first;
        ++labelIndex;
    }

    return labels;
}

float DecisionTreeAlgorithm::decisiontree(cv::Mat& trainingData, cv::Mat& trainingClasses, cv::Mat& testData, cv::Mat& testClasses)
{
    CvDTree dtree;

    uint8_t *data = new uint8_t[11]; // initial row data for type mask
    for(int i = 0; i < 11; ++i)
    {
        data[i] = 0;
    }

    cv::Mat var_type(11, 1, CV_8UC1, data);

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
    cv::Mat predicted(testClasses.rows, 1, CV_32F); // zu gross....
    predicted.zeros(testClasses.rows, 1, CV_32F);
    for (int i = 0; i < testData.rows; i++)
    {
        const cv::Mat sample = testData.row(i);
        CvDTreeNode* prediction = dtree.predict(sample);
        predicted.at<float> (i, 0) = prediction->value;
    }

    std::cout << "testClasses = " << testClasses << std::endl;
    std::cout << "predicted = " << predicted << std::endl;
    //std::cout << "Accuracy_{TREE} = " << evaluate(predicted, testClasses) << std::endl;
    std::cout << "Var importance mat = " << dtree.getVarImportance() << std::endl;

    outInfo("prediction value: " <<  predicted.at<float>(0,0));
    delete[] data;

    return predicted.at<float>(0,0);
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
