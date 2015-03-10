#ifndef __LEARNANNOTATIONSTORAGE_H__
#define __LEARNANNOTATIONSTORAGE_H__

#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <iai_rs/types/all_types.h>
//RS
#include <iai_rs/scene_cas.h>
#include <iai_rs/io/Storage.h>
#include <iai_rs/util/time.h>

// MP includes
#include <containers/Geometry.h>
#include <containers/GroundTruth.h>
#include <containers/SemanticColor.h>
#include <containers/LearningAnnotation.h>
#include <MPIdentifiable.h>

using namespace uima;

namespace rs_log_learn
{

/*
 * Manager used for reading out all learned annotations from db containing
 * learning annotations
 */
class LearnAnnotationStorage
{
private:
    std::vector<iai_rs::Learning> allAnnotations;
    int sceneNo = 0;

    std::string learning_host;
    std::string learning_db;

    iai_rs::Storage storage;
    std::vector<uint64_t> frames; // scene timestamps
    std::vector<iai_rs::SceneCas*> learningScenes; // only used for extraction on first frame
    std::map<uint64_t, std::vector<iai_rs::Cluster>> timestampedClusters; // only used for extraction on first frame
    std::vector<Geometry> learningGeometry; // here
    // TODO: container: own MPIdentifiable that can hold different annotation data between frames + metadata

    iai_rs::SceneCas* loadScene(uint64_t timestamp, CAS &tcas);
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
