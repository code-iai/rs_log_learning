#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
// IAI includes
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>

// MP includes
#include <rs_log_learning/types/all_types.h>
#include <rs_log_learning/LearnAnnotationStorage.h>
#include <rs_log_learning/MPCore.h>

using namespace uima;
using namespace rs_log_learning;

class MindPalaceAnnotator: public Annotator
{
private:
    std::vector<rs_log_learning::Learning> allAnnotations;

    MPCore mp;

    std::string learning_host;
    std::string learning_db;
    std::string mode;
    std::string algorithm;

public:

    TyErrorId initialize(AnnotatorContext &ctx)
    {
        outInfo("initialize");
        ctx.extractValue("learningHost", learning_host);
        ctx.extractValue("learningDB", learning_db);
        ctx.extractValue("mode", mode);
        ctx.extractValue("algorithm", algorithm);

        ConfigParams parameters;
        parameters.learningHost = learning_host;
        parameters.learningDB = learning_db;
        parameters.mode = mode;
        parameters.algorithm = algorithm;

        mp.setConfigParams(parameters);

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
        return UIMA_ERR_NONE;
    }

    TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
    {
        outInfo("process start");
        outInfo("creating MPCore");

        rs::StopWatch clock;

        mp.process(tcas);

        outInfo("took: " << clock.getTime() << " ms.");
        return UIMA_ERR_NONE;
    }

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(MindPalaceAnnotator)
