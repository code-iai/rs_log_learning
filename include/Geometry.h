#ifndef __LEARNGEOMETRY_H__
#define __LEARNGEOMETRY_H__

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
  Geometry(iai_rs::Geometry geo);
  Geometry() {};
  ~Geometry();

  std::string getSize() {return size;}
  double getBoundingBoxWidth() {return boundingBox.width;}
  double getBoundingBoxHeight(){return boundingBox.height;}
  double getBoundingBoxDepth() {return boundingBox.depth;}
  double getBoundingBoxVolume(){return boundingBox.volume;}
};

}

#endif //__LEARNGEOMETRY_H__
