#ifndef __SEMANTICCOLOR_H__
#define __SEMANTICCOLOR_H__

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
 * simple container for rs::SemanticColor object data to keep them between frames
 */
class SemanticColor
{
private:
    std::map<std::string, float> sColorMapping;

public:
    SemanticColor(iai_rs::SemanticColor sColor);
    SemanticColor() {};
    ~SemanticColor();

    std::map<std::string, float> getColorMapping();
};

}

#endif //__SEMANTICCOLOR_H__
