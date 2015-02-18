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
	annotate(tcas);
	learn(tcas);
}

/*
 * Learn...
 */
void MPCore::learn(uima::CAS &tcas)
{
	// set only a test string for now
	iai_rs::SceneCas cas(tcas);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
	cas.getPointCloud(*cloud_ptr);
	outInfo("Cloud size: " << cloud_ptr->points.size());


	iai_rs::Learning lrn = iai_rs::create<iai_rs::Learning>(tcas);

	lrn.test_learn_string.set("testLRNString");

	std::vector<iai_rs::Cluster> clusters;
	cas.getScene().identifiables.filter(clusters);


	for(int i = 0; i < clusters.size(); ++i)
	{
		std::stringstream ssLearn;
		ssLearn << "testLRNString SceneNo: " << sceneNo << "  ClusterNo: " << i;

		lrn.test_learn_string.set(ssLearn.str());
		clusters[i].annotations.append(lrn);
	}
	sceneNo++;
}

/*
 * Annotate...
 */
void MPCore::annotate(uima::CAS &tcas)
{
	//learnAS_.test_get_stuff(tcas);
	if(!learnDBloaded)
	{
		learnIdentifiables_ = learnAS_.extractLearnIdentifiables(tcas);
	}

	// testing output
    for(std::vector<MPIdentifiable>::iterator it = learnIdentifiables_.begin();
        it != learnIdentifiables_.end(); ++it)
    {
    	std::string size = it->getGeometry().getSize();
    	double w = it->getGeometry().getBoundingBoxWidth();
    	double d = it->getGeometry().getBoundingBoxDepth();
    	double h = it->getGeometry().getBoundingBoxHeight();
    	double v = it->getGeometry().getBoundingBoxVolume();
        outInfo("Vector geometry size: " << size);
        outInfo("Vector geometry boundingbox w*d*h=v: " << w << "*" << d << "*" << h << "=" << v);
    }
}

} /* namespace rs_log_learn */
