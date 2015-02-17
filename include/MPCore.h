/*
 * MPCore.h
 *
 *  Created on: Feb 17, 2015
 *      Author: andre
 */

#ifndef RS_LOG_LEARN_SRC_MPCORE_H_
#define RS_LOG_LEARN_SRC_MPCORE_H_

#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <iai_rs/types/all_types.h>

// IAI includes
#include <iai_rs/scene_cas.h>
#include <iai_rs/util/time.h>
#include <iai_rs/DrawingAnnotator.h>

// MP FeatureStructures
#include <Geometry.h>
// MP includes
#include <LearnAnnotationStorage.h>

namespace rs_log_learn
{

/*
 * Configuration Parameters set on initialization
 */
struct ConfigParams
{
	std::string learningHost;
	std::string learningDB;
};


/*
 * MindPalace core handler
 */
class MPCore
{
public:
	MPCore(ConfigParams params);
	virtual ~MPCore();

	const ConfigParams& getConfigParams() const { return configParams_; }
	void setConfigParams(const ConfigParams& configParams) { configParams_ = configParams; }

	void process(uima::CAS &tcas);

	void learn(uima::CAS &tcas);    // write leaning data to db and mem
	void annotate(uima::CAS &tcas); // annotate cas with learning data

private:
	ConfigParams configParams_;
	LearnAnnotationStorage learnAS_;
	int sceneNo = 0;

	void initialize();

};

} /* namespace rs_log_learn */

#endif /* RS_LOG_LEARN_SRC_MPCORE_H_ */
