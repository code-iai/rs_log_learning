#include <LearnAnnotationStorage.h>


#define DB_HOST "localhost"
#define DB_NAME "Scenes"

using namespace uima;
using namespace rs_log_learn;

LearnAnnotationStorage::LearnAnnotationStorage(const std::string host, const std::string db)
    : learning_host(host),
      learning_db(db)
{
    storage = iai_rs::Storage(learning_host, learning_db);
}

LearnAnnotationStorage::~LearnAnnotationStorage()
{
}


/*
 * Testing method for now...
 */
void LearnAnnotationStorage::test_get_stuff(CAS &tcas)
{
    iai_rs::util::StopWatch clock;

    if(!db_loaded)
    {
        extractScenes(tcas);
        extractClusters();
    }
    db_loaded = true;



    for(std::map<uint64_t, std::vector<iai_rs::Cluster>>::iterator it=timestampedClusters.begin();
        it != timestampedClusters.end(); ++it)
    {
        outInfo("TimeStamp: " << it->first << "  No of clusters: " << it->second.size());
        for(std::vector<iai_rs::Cluster>::iterator cit = it->second.begin();
            cit != it->second.end(); ++cit)
        {
          std::vector<iai_rs::Geometry> geometry;
          std::vector<iai_rs::Learning> learning;
          cit->annotations.filter(geometry);
          cit->annotations.filter(learning);

          outInfo("Geometry size: " << geometry.at(0).size.get());
          outInfo("Learn test Str: " << learning.at(0).test_learn_string.get());
        }
    }


    outInfo(clock.getTime() << " ms.");
}


// preprocessing:
// * load from db
// * extract clusters
// * save learn strings, ids etc. to save time

void LearnAnnotationStorage::extractScenes(CAS &tcas)
{
    outInfo("Host: " << learning_host);
    outInfo("DB: " << learning_db);
    storage.getScenes(frames);

    if(frames.empty())
    {
        outError("No frames found in learning DB");
    }

    outInfo("Frame IDs size: " << frames.size());


    for(int i = 0; i < frames.size(); ++i)
    {
        iai_rs::SceneCas* sc = loadScene(frames[i], tcas);
        learningScenes.push_back(sc);
    }
    outInfo("Loaded scenes size: " << learningScenes.size());
}

void LearnAnnotationStorage::extractClusters()
{
    long cluster_size = 0;

    for(int i = 0; i < learningScenes.size(); ++i)
    {
        std::vector<iai_rs::Cluster> clusters;

        learningScenes[i]->getScene().identifiables.filter(clusters);
        uint64_t ts = learningScenes[i]->getScene().timestamp.get();

        outInfo("TimeStamp: " << ts);

        timestampedClusters[ts] = clusters;

        for(std::vector<iai_rs::Cluster>::iterator it = clusters.begin();
            it != clusters.end(); ++it)
        {
            cluster_size += sizeof(*it);
        }
    }

    outInfo("SizeOf all clusters: " << cluster_size/1024 << "k");
}

/*
 * Get scene using an empty cas
 */
iai_rs::SceneCas* LearnAnnotationStorage::loadScene(uint64_t timestamp, CAS &tcas) // constcorr
{
    // creation of new CAS only possible from AE?
    // other way instead of
    //   engine = uima::Framework::createAnalysisEngine(file.c_str(), errorInfo);
    //   engine->newCAS()
    storage.loadScene(*tcas.getBaseCas(), timestamp);
    iai_rs::SceneCas* newcas = new iai_rs::SceneCas(tcas);
    return newcas;
}
