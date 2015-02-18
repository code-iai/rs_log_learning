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
        saveFeatureStructures();
    }
    db_loaded = true;

    // in all processed frames. see, if we can keep the data saved in memory
    for(std::vector<Geometry>::iterator git = learningGeometry.begin();
        git != learningGeometry.end(); ++git)
    {
        outInfo("Vector geometry size: " << git->getSize());
        outInfo("Vector geometry boundingbox w*d*h=v: " << git->getBoundingBoxWidth() << "*"
        												<< git->getBoundingBoxDepth() << "*"
														<< git->getBoundingBoxHeight() << "="
														<< git->getBoundingBoxVolume());
    }
    outInfo(clock.getTime() << " ms.");
}


/*
 * Extract identifiables from learning db
 */
std::vector<MPIdentifiable> LearnAnnotationStorage::extractLearnIdentifiables(CAS &tcas)
{
	std::vector<MPIdentifiable> result;

	// load data from db
	extractScenes(tcas);
	extractClusters();

	// construct new identifiable objects of gathered data
	for(std::map<uint64_t, std::vector<iai_rs::Cluster>>::iterator it=timestampedClusters.begin();
			it != timestampedClusters.end(); ++it)
		{
			outInfo("Clusters in map size: " << it->second.size());

			outInfo("TimeStamp: " << it->first << "  No of clusters: " << it->second.size());
			for(std::vector<iai_rs::Cluster>::iterator cit = it->second.begin();
				cit != it->second.end(); ++cit)
			{
			  std::vector<iai_rs::Geometry> geometry;
			  std::vector<iai_rs::Learning> learning;
			  cit->annotations.filter(geometry);
			  cit->annotations.filter(learning);

			  // as soon as first pipeline run is finished, data in clusters gets cleared :(
			  if(geometry.empty())
				  outError("geometry empty");
			  else
			  {
				  MPIdentifiable ident(it->first);
				  ident.setGeometry(Geometry(geometry.at(0)));
				  // TODO: extract more
				  result.push_back(ident);
			  }
			  if(learning.empty())
				  outError("learning empty");
			  else
				outInfo("Learn test Str: " << learning.at(0).test_learn_string.get());
			}
		}

	return result;
}

// preprocessing:
// * load from db
// * extract clusters
// * save learn strings, ids etc. to save time

/*
 * Save annotations on the first run
 * ... saving only geometry for now
 */
void LearnAnnotationStorage::saveFeatureStructures()
{
	// grab a bit of data as an example on the first frame
	for(std::map<uint64_t, std::vector<iai_rs::Cluster>>::iterator it=timestampedClusters.begin();
		it != timestampedClusters.end(); ++it)
	{
		outInfo("Clusters in map size: " << it->second.size());

		outInfo("TimeStamp: " << it->first << "  No of clusters: " << it->second.size());
		for(std::vector<iai_rs::Cluster>::iterator cit = it->second.begin();
			cit != it->second.end(); ++cit)
		{
		  std::vector<iai_rs::Geometry> geometry;
		  std::vector<iai_rs::Learning> learning;
		  cit->annotations.filter(geometry);
		  cit->annotations.filter(learning);

		  // as soon as first pipeline run is finished, data in clusters gets cleared :(
		  if(geometry.empty())
			  outError("geometry empty");
		  else
		  {
			  iai_rs::Geometry geoCopy(geometry.at(0));
			  // copy data to own container to not loose it between frames
			  learningGeometry.push_back(Geometry(geometry.at(0)));
		  }
		  if(learning.empty())
			  outError("learning empty");
		  else
			outInfo("Learn test Str: " << learning.at(0).test_learn_string.get());
		}
	}
}

/*
 * Extract the scenes from the learning db using the extracted
 * frame timestamps and a dummy CAS
 */
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

/*
 * Extract a vector of clusters within the scenes.
 * They are stored in a map together with their corresponding frame timestamp
 */
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
 * Get scene using an empty/dummy cas
 */
iai_rs::SceneCas* LearnAnnotationStorage::loadScene(uint64_t timestamp, CAS &tcas) // constcorr
{
    // creation of new CAS only possible from AE?
    // other way instead of
    //   engine = uima::Framework::createAnalysisEngine(file.c_str(), errorInfo);
    //   engine->newCAS()

    // bad, as at least the first CAS processed is overwritten with stuff from the learning db
    storage.loadScene(*tcas.getBaseCas(), timestamp);
    iai_rs::SceneCas* newcas = new iai_rs::SceneCas(tcas);
    return newcas;
}
