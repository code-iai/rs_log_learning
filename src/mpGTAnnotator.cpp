#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <iai_rs/types/all_types.h>

// IAI includes
#include <iai_rs/scene_cas.h>
#include <iai_rs/util/time.h>
#include <iai_rs/DrawingAnnotator.h>

// MP includes
#include <mpGTui.h>


using namespace uima;
using namespace rs_log_learn;


class mpGTAnnotator : public Annotator
{
private:


public:

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

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

    outInfo("spawn UI");

    const char *argv[] = {"GroundTruth Annotator", NULL};
	int argc = sizeof(argv) / sizeof(char*) - 1;
	char** argv_gtk_eat_shit = const_cast<char**> (&argv[0]);

	Gtk::Main kit(argc, argv_gtk_eat_shit);
	mpGTui ui;
    Gtk::Main::run(ui);

    outInfo("took: " << clock.getTime() << " ms.");
    return UIMA_ERR_NONE;
  }

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(mpGTAnnotator)
