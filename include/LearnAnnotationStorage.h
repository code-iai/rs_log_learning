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

  iai_rs::Storage storage;
  std::vector<uint64_t> frames;
  std::string learning_host;
  std::string learning_db;


public:
  LearnAnnotationStorage(const std::string host, const std::string db);
  LearnAnnotationStorage() {};
  ~LearnAnnotationStorage();

  void test_get_stuff();
  CAS loadScene(uint64_t timestamp); // TODO: make private

};

}

#endif //__LEARNANNOTATIONSTORAGE_H__
