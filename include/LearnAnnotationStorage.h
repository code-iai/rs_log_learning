#ifndef __LEARNANNOTATIONSTORAGE_H__
#define __LEARNANNOTATIONSTORAGE_H__

#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <iai_rs/types/all_types.h>
//RS
#include <iai_rs/scene_cas.h>
#include <iai_rs/io/Storage.h>
#include <iai_rs/util/time.h>

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
  bool db_loaded = false; // learning db has been loaded once initially

  std::string learning_host;
  std::string learning_db;

  iai_rs::Storage storage;
  std::vector<uint64_t> frames;
  std::vector<iai_rs::SceneCas*> learningScenes;
  std::map<uint64_t, std::vector<iai_rs::Cluster>> timestampedClusters;

  iai_rs::SceneCas* loadScene(uint64_t timestamp, CAS &tcas);
  void extractScenes(CAS &tcas);
  void extractClusters();


public:
  LearnAnnotationStorage(const std::string host, const std::string db);
  LearnAnnotationStorage() {};
  ~LearnAnnotationStorage();

  void test_get_stuff(CAS &tcas);
};

}

#endif //__LEARNANNOTATIONSTORAGE_H__