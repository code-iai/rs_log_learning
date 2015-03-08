/*
 * MPCore.cpp
 *
 *  Created on: Feb 17, 2015
 *      Author: andre
 */

#include "MPCore.h"

namespace rs_log_learn
{

MPCore::MPCore(ConfigParams params)
{
    this->configParams_ = params;
}

MPCore::~MPCore()
{
    // TODO Auto-generated destructor stub
}

void MPCore::initialize()
{
    learnAS_ = LearnAnnotationStorage(this->configParams_.learningHost,
            this->configParams_.learningDB);
}

void MPCore::process(uima::CAS &tcas)
{
    initialize();

    outInfo("------------------------");
    outInfo("Learning Host = " << this->configParams_.learningHost);
    outInfo("Learning DB   = " << this->configParams_.learningDB);
    outInfo("------------------------");

    // TODO: decide whether to use learn and/or annotate based on settings
    // if first cas of engine, dont annotate that one, dbloaded==false
    annotate(tcas);
    //learn(tcas);
}

/*
 * Learn...
 */
void MPCore::learn(uima::CAS &tcas)
{
    // only load from learning db on first frame.
    // after this everything will be in memory for the pipeline run
    // FIXME: still burning the first CAS this way
    if (!learnDBloaded)
    {
        outInfo("loading learnDB");
        learnIdentifiables_ = learnAS_.extractLearnIdentifiables(tcas);
        learnDBloaded = true;
    }

    // testing output
    for (std::vector<MPIdentifiable>::iterator it = learnIdentifiables_.begin();
            it != learnIdentifiables_.end(); ++it)
    {
        std::string size = it->getGeometry().getSize();
        double w = it->getGeometry().getBoundingBoxWidth();
        double d = it->getGeometry().getBoundingBoxDepth();
        double h = it->getGeometry().getBoundingBoxHeight();
        double v = it->getGeometry().getBoundingBoxVolume();
        outInfo("GroundTruth - global: " << it->getGroundTruth().getGlobaltGt()
                << " / shape: " << it->getGroundTruth().getShape());
        outInfo("Vector geometry size: " << size);
        outInfo("Vector geometry boundingbox w*d*h=v: " << w << "*" << d << "*" << h << "=" << v);
        outInfo("Learning Annotation - name: " << it->getLearningAnnotation().getLearnedName())
                << " / shape: " << it->getLearningAnnotation().getShape();
    }
}

/*
 * Annotate...
 */
void MPCore::annotate(uima::CAS &tcas)
{
    // set only a test string for now
    iai_rs::SceneCas cas(tcas);
    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(
    //        new pcl::PointCloud<pcl::PointXYZRGBA>);
    //cas.getPointCloud(*cloud_ptr);
    //outInfo("Cloud size: " << cloud_ptr->points.size());

    iai_rs::Learning lrn = iai_rs::create<iai_rs::Learning>(tcas);

    //lrn.name.set("testLRNString");
    //lrn.name.set("Box (test)");

    std::vector<iai_rs::Cluster> clusters;
    cas.getScene().identifiables.filter(clusters);

    // mlpack test here

    for (int i = 0; i < clusters.size(); ++i)
    {
        std::stringstream ssLearn;
        ssLearn << "testLRNString SceneNo: " << sceneNo << "  ClusterNo: " << i;

        lrn.name.set(ssLearn.str());
        lrn.shape.set("Box (test)");
        clusters[i].annotations.append(lrn);
    }
    sceneNo++;
}

} /* namespace rs_log_learn */
