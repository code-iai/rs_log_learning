#include <Geometry.h>

using namespace uima;
using namespace rs_log_learn;

Geometry::Geometry(iai_rs::Geometry geo)
{
    iai_rs::BoundingBox3D iai_bb(geo.boundingBox.get());

    size = geo.size.get();
    boundingBox.width = iai_bb.width.get();
    boundingBox.height = iai_bb.height.get();
    boundingBox.depth = iai_bb.depth.get();
    boundingBox.volume = iai_bb.volume.get();
}

Geometry::~Geometry()
{
}

