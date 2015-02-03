#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <iai_rs/types/all_types.h>
//RS
#include <iai_rs/scene_cas.h>
#include <iai_rs/util/time.h>
#include <iai_rs/DrawingAnnotator.h>
//RS_LOG_LEARN
#include <LearnAnnotationStorage.h>

using namespace uima;


class SimpleLearnAnnotator : public Annotator
{
private:
  float test_param;
  std::vector<iai_rs::Learning> allAnnotations;
  int sceneNo = 0;

  std::string learning_host;
  std::string learning_db;

  rs_log_learn::LearnAnnotationStorage learnas;

public:

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    ctx.extractValue("test_param", test_param);
    ctx.extractValue("learningHost", learning_host);
    ctx.extractValue("learningDB", learning_db);

    learnas = rs_log_learn::LearnAnnotationStorage(learning_host, learning_db);
    return UIMA_ERR_NONE;
  }

  TyErrorId typeSystemInit(TypeSystem const &type_system)
  {
    outInfo("typeSystemInit");
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process()");
    this->processWithLock(tcas, res_spec);
    return  UIMA_ERR_NONE;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    iai_rs::util::StopWatch clock;
    iai_rs::SceneCas cas(tcas);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

    outInfo("Test param =  " << test_param);
    outInfo("------------------------");
    outInfo("Learning Host = " << learning_host);
    outInfo("Learning DB   = " << learning_db);
    outInfo("------------------------");

    learnas.test_get_stuff();

    cas.getPointCloud(*cloud_ptr);

    iai_rs::Learning lrn = iai_rs::create<iai_rs::Learning>(tcas);

    lrn.test_learn_string.set("testLRNString");

    std::vector<iai_rs::Cluster> clusters;
    cas.getScene().identifiables.filter(clusters);


    for(int i = 0; i < clusters.size(); ++i)
    {
        std::stringstream ssLearn;
        ssLearn << "testLRNString SceneNo: " << sceneNo << "  ClusterNo: " << i;

        lrn.test_learn_string.set(ssLearn.str());
        clusters[i].annotations.append(lrn);
    }
    sceneNo++;

    outInfo("Cloud size: " << cloud_ptr->points.size());
    outInfo("took: " << clock.getTime() << " ms.");
    return UIMA_ERR_NONE;
  }

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(SimpleLearnAnnotator)
