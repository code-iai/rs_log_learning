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

// MP includes
#include <LearnAnnotationStorage.h>
#include <MPIdentifiable.h>
#include <algorithms/NearestNeighborAlgorithm.h>

namespace rs_log_learn
{

/*
 * Configuration Parameters set on initialization
 */
struct ConfigParams
{
    std::string learningHost;
    std::string learningDB;
    std::string mode; // switch to an enum
    std::string algorithm;
};

/*
 * MindPalace core handler
 */
class MPCore
{
public:
    MPCore(ConfigParams params);
    MPCore() {};
    virtual ~MPCore();

    const ConfigParams& getConfigParams() const
    {
        return configParams_;
    }
    void setConfigParams(const ConfigParams& configParams)
    {
        configParams_ = configParams;
    }

    void process(uima::CAS &tcas);

    void learn(uima::CAS &tcas);    // write leaning data to db and mem
    void annotate(uima::CAS &tcas); // annotate cas with learning data

private:
    ConfigParams configParams_;
    LearnAnnotationStorage learnAS_;
    std::vector<MPIdentifiable> learnIdentifiables_;

    MPIdentifiable extractIdentifiableFromCluster(iai_rs::Cluster cluster);

    int sceneNo = 0;
    bool learnDBloaded = false;

    void initialize();

};

} /* namespace rs_log_learn */

#endif /* RS_LOG_LEARN_SRC_MPCORE_H_ */
