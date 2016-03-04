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
#include <algorithms/DecisionTreeAlgorithm.h>

namespace rs_log_learn
{

#define CONFIDENCE_THRESHOLD 0.5
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
    void train(uima::CAS &tcas); // annotate cas with learning data

private:
    ConfigParams configParams_;
    LearnAnnotationStorage learnAS_;
    std::vector<MPIdentifiable> learnIdentifiables_;
    std::vector<MPIdentifiable> additionalDTIdentifiables_;

    MPIdentifiable extractIdentifiableFromCluster(rs::Cluster cluster);

    int sceneNo = 0;
    bool learnDBloaded = false;

    void initialize();
    void loadDB(uima::CAS &tcas);

    int pushed_back_idents = 0;
    int delete_next_turn = 0;
    bool additionalsAvailable = false;

};

} /* namespace rs_log_learn */

#endif /* RS_LOG_LEARN_SRC_MPCORE_H_ */
