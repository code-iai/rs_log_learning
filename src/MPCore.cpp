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
	learnAS_.test_get_stuff(tcas);
}

} /* namespace rs_log_learn */
