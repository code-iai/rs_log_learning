#ifndef __SEMANTICCOLOR_H__
#define __SEMANTICCOLOR_H__

#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/io/Storage.h>
#include <rs/utils/time.h>

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
    SemanticColor(rs::SemanticColor sColor);
    SemanticColor() {};
    ~SemanticColor();

    std::map<std::string, float> getColorMapping();
};

}

#endif //__SEMANTICCOLOR_H__
