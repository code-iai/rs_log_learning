/*
 * DecisionTreeAlgorithm.cpp
 *
 *  Created on: Mar 17, 2015
 *      Author: andre
 */

#include <rs_log_learning/algorithms/DecisionTreeAlgorithm.h>

namespace rs_log_learning
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
  rs::StopWatch clock;

  float *initialTrainingData = new float[10 * referenceSet.size()]; // data for training
  float *testData = new float[10]; // query data

  mm = (int)round((float)20 / referenceSet.size() * 10); // normalize
  outInfo("mm: " << mm);

  for(int i = 0; i < referenceSet.size(); ++i)
  {
    initialTrainingData[i * 10] = (float)referenceSet[i].getGeometry().getBoundingBoxVolume();
    if(!referenceSet[i].getSemColor().getColorMapping().empty())
    {
      initialTrainingData[i * 10 + 1] = referenceSet[i].getSemColor().getColorMapping()["white"];
      initialTrainingData[i * 10 + 2] = referenceSet[i].getSemColor().getColorMapping()["yellow"];
      initialTrainingData[i * 10 + 3] = referenceSet[i].getSemColor().getColorMapping()["red"];
      initialTrainingData[i * 10 + 4] = referenceSet[i].getSemColor().getColorMapping()["black"];
      initialTrainingData[i * 10 + 5] = referenceSet[i].getSemColor().getColorMapping()["grey"];
      initialTrainingData[i * 10 + 6] = referenceSet[i].getSemColor().getColorMapping()["blue"];
      initialTrainingData[i * 10 + 7] = referenceSet[i].getSemColor().getColorMapping()["green"];
      initialTrainingData[i * 10 + 8] = referenceSet[i].getSemColor().getColorMapping()["magenta"];
      initialTrainingData[i * 10 + 9] = referenceSet[i].getSemColor().getColorMapping()["cyan"];
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

  float *trainingData = new float[mm * 10 * referenceSet.size()]; // final training data
  for(int i = 0; i < mm; ++i)
  {
    std::copy(initialTrainingData, initialTrainingData + 10 * referenceSet.size(), trainingData + i * 10 * referenceSet.size());
  }

  cv::Mat testMat(1, 10, CV_32FC1, testData);
  cv::Mat trainingMat(mm * referenceSet.size(), 10, CV_32FC1, trainingData);

  //std::cout << "TrainingData: " << trainingMat << std::endl;
  //std::cout << "testData: " << testMat << std::endl;

  cv::Mat trainingClasses = labelData(referenceSet);
  cv::Mat testClasses = labelData(referenceSet);

  //std::cout << "trainingClasses: " << trainingClasses << std::endl;

  float resultValue = decisiontree(trainingMat, trainingClasses, testMat, testClasses);
  float confidence = 1 - abs(resultValue - (size_t)round(resultValue));
  size_t resultIndex = (size_t)round(resultValue);

  LearningAnnotation lrn;

  if(referenceSet[resultIndex].getGroundTruth().getGlobaltGt().empty())
  {
    lrn.setLearnedName(referenceSet[resultIndex].getLearningAnnotation().getLearnedName());
    lrn.setShape(referenceSet[resultIndex].getLearningAnnotation().getShape());
  }
  else
  {
    lrn.setLearnedName(referenceSet[resultIndex].getGroundTruth().getGlobaltGt());
    lrn.setShape(referenceSet[resultIndex].getGroundTruth().getShape());
  }

  lrn.setConfidence(confidence);

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
  if(referenceSet[0].getGroundTruth().getGlobaltGt().empty())
  {
    outInfo("no gt available for all clusters, adding learning data to reference set");
  }
  for(int i = 0; i < referenceSet.size(); ++i)
  {
    NameShape ns;
    if(referenceSet[i].getGroundTruth().getGlobaltGt().empty())
    {
      mixedLearn = true;
      ns.name = referenceSet[i].getLearningAnnotation().getLearnedName();
      ns.shape = referenceSet[i].getLearningAnnotation().getShape();
    }
    else
    {
      // gt data is available
      ns.name = referenceSet[i].getGroundTruth().getGlobaltGt();
      ns.shape = referenceSet[i].getGroundTruth().getShape();
    }
    indexToAnnotationData[i] = ns;
  }
  outInfo("classification map size: " << indexToAnnotationData.size());

  int labelIndex = 0;
  cv::Mat labels(indexToAnnotationData.size()*mm, 1, CV_32FC1);
  for(std::map<int, NameShape>::iterator it = indexToAnnotationData.begin(); it != indexToAnnotationData.end(); ++it)
  {
    labels.at<float>(labelIndex, 0) = (float)it->first;
    for(int i = 0; i < mm; ++i)
    {
      labels.at<float>(labelIndex + indexToAnnotationData.size() * i, 0) = (float)it->first;
    }
    ++labelIndex;
  }

  return labels;
}

float DecisionTreeAlgorithm::decisiontree(cv::Mat &trainingData, cv::Mat &trainingClasses, cv::Mat &testData, cv::Mat &testClasses)
{
  CvDTree dtree;

  uint8_t *data = new uint8_t[11]; // initial row data for type mask
  for(int i = 0; i < 11; ++i)
  {
    data[i] = 0;
  }

  cv::Mat var_type(11, 1, CV_8UC1, data);

  outInfo("starting training");


  dtree.train(trainingData, CV_ROW_SAMPLE, trainingClasses, cv::Mat(), cv::Mat(), var_type, cv::Mat(),
              CvDTreeParams(8,    // max depth
                            5,    // min sample count
                            0,    // regression N/A
                            true, // compute surrogate split for missing data
                            15,   // max number of categories
                            1,    // def: 10 number of cross validation folds
                            true, // 1SE rule -> smaller tree
                            true, // throw away pruned tree branches
                            0));  // no priors
  cv::Mat predicted(testClasses.rows, 1, CV_32F); // zu gross....
  predicted.zeros(testClasses.rows, 1, CV_32F);

  outInfo("predicting input query");
  for(int i = 0; i < testData.rows; i++)
  {
    const cv::Mat sample = testData.row(i);
    CvDTreeNode *prediction = dtree.predict(sample);
    predicted.at<float> (i, 0) = prediction->value;
  }

  //std::cout << "testClasses = " << testClasses << std::endl;
  //std::cout << "predicted = " << predicted << std::endl;
  //std::cout << "Accuracy_{TREE} = " << evaluate(predicted, testClasses) << std::endl;
  //std::cout << "Var importance mat = " << dtree.getVarImportance() << std::endl;

  outInfo("prediction value: " <<  predicted.at<float>(0, 0));
  delete[] data;

  return predicted.at<float>(0, 0);
}

} /* namespace rs_log_learn */
