#ifndef __LEARNANNOTATIONSTORAGE_H__
#define __LEARNANNOTATIONSTORAGE_H__

#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
#include <rs_log_learning/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/io/Storage.h>
#include <rs/utils/time.h>

// MP includes
#include <rs_log_learning/containers/Geometry.h>
#include <rs_log_learning/containers/GroundTruth.h>
#include <rs_log_learning/containers/SemanticColor.h>
#include <rs_log_learning/containers/LearningAnnotation.h>
#include <rs_log_learning/MPIdentifiable.h>

using namespace uima;

namespace rs_log_learning
{

/*
 * Manager used for reading out all learned annotations from db containing
 * learning annotations
 */
class LearnAnnotationStorage
{
private:
    std::vector<rs_log_learning::Learning> allAnnotations;
    int sceneNo = 0;

    std::string learning_host;
    std::string learning_db;

    rs::Storage storage;
    std::vector<uint64_t> frames; // scene timestamps
    std::vector<rs::SceneCas*> learningScenes; // only used for extraction on first frame
    std::map<uint64_t, std::vector<rs::Cluster>> timestampedClusters; // only used for extraction on first frame
    std::vector<Geometry> learningGeometry; // here
    // TODO: container: own MPIdentifiable that can hold different annotation data between frames + metadata

    rs::SceneCas* loadScene(uint64_t timestamp, CAS &tcas);
    void extractScenes(CAS &tcas);
    void extractClusters();

public:
    LearnAnnotationStorage(const std::string host, const std::string db);
    LearnAnnotationStorage()
    {
    }
    ;
    ~LearnAnnotationStorage();

    std::vector<MPIdentifiable> extractLearnIdentifiables(CAS &tcas);
};

}

#endif //__LEARNANNOTATIONSTORAGE_H__
