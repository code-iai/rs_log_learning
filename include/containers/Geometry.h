#ifndef __LEARNGEOMETRY_H__
#define __LEARNGEOMETRY_H__

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
 * simple container for rs::Geometry object data to keep them between frames
 */
class Geometry
{
private:
    std::string size;

    struct BoundingBox
    {
        double width;
        double depth;
        double height;
        double volume;
    };

    BoundingBox boundingBox;

public:
    Geometry(rs::Geometry geo);
    Geometry()
    {
    }
    ;
    ~Geometry();

    std::string getSize()
    {
        return size;
    }
    double getBoundingBoxWidth()
    {
        return boundingBox.width;
    }
    double getBoundingBoxHeight()
    {
        return boundingBox.height;
    }
    double getBoundingBoxDepth()
    {
        return boundingBox.depth;
    }
    double getBoundingBoxVolume()
    {
        return boundingBox.volume;
    }
};

}

#endif //__LEARNGEOMETRY_H__
