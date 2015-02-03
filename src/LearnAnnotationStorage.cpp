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
void LearnAnnotationStorage::test_get_stuff()
{
    iai_rs::util::StopWatch clock;

    outInfo("Host: " << learning_host);
    outInfo("DB: " << learning_db);
    storage.getScenes(frames);

    std::stringstream ssLearn;

    if(frames.empty())
    {
        outError("No frames in learned DB");
    }

    outInfo("Scene IDs size: " << frames.size());

    for(int i = 0; i < frames.size(); ++i)
    {
        ssLearn << frames[i] << ", ";
    }
    ssLearn << ";";
    outInfo("Got frame timestamps from DB: " << ssLearn.str());

    std::vector<CAS> allScenes;
    for(int i = 0; i < frames.size(); ++i)
    {
        //allScenes.push_back(loadScene(frames[i]));
    }
    outInfo("Loaded scenes size: " << allScenes.size());

    outInfo(clock.getTime() << " ms.");
}

/*
 * Get scene using an empty cas
 */
CAS LearnAnnotationStorage::loadScene(uint64_t timestamp) // constcorr
{
    // creation of new CAS only possible from AE?
    // other way instead of
    //   engine = uima::Framework::createAnalysisEngine(file.c_str(), errorInfo);
    //   engine->newCAS()

    //return new CAS();
}
