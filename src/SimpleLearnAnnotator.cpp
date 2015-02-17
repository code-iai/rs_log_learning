#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <iai_rs/types/all_types.h>

// IAI includes
#include <iai_rs/scene_cas.h>
#include <iai_rs/util/time.h>
#include <iai_rs/DrawingAnnotator.h>

// MP includes
#include <LearnAnnotationStorage.h>
#include <MPCore.h>


using namespace uima;
using namespace rs_log_learn;


class SimpleLearnAnnotator : public Annotator
{
private:
  float test_param;
  std::vector<iai_rs::Learning> allAnnotations;

  std::string learning_host;
  std::string learning_db;

public:

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    ctx.extractValue("test_param", test_param);
    ctx.extractValue("learningHost", learning_host);
    ctx.extractValue("learningDB", learning_db);

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
    outInfo("creating MPCore");

    iai_rs::util::StopWatch clock;

    ConfigParams parameters;
    parameters.learningHost = learning_host;
    parameters.learningDB   = learning_db;

    MPCore mp(parameters);

    mp.process(tcas);

    outInfo("took: " << clock.getTime() << " ms.");
    return UIMA_ERR_NONE;
  }

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(SimpleLearnAnnotator)
