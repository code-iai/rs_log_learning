/*
 * MPIdentifiable.h
 *
 *  Created on: Feb 18, 2015
 *      Author: andre
 */

#ifndef RS_LOG_LEARN_SRC_MPIDENTIFIABLE_H_
#define RS_LOG_LEARN_SRC_MPIDENTIFIABLE_H_

// MP includes
#include <containers/Geometry.h>
#include <containers/LearningAnnotation.h>
#include <containers/GroundTruth.h>

namespace rs_log_learn
{

/*
 * MPs own simple identifiable structure to keep extracted data between pipeline runs
 */
class MPIdentifiable
{
public:
    MPIdentifiable(uint64_t frame_timestamp);
    virtual ~MPIdentifiable();

    uint64_t getFrameTimestamp() const
    {
        return frame_timestamp_;
    }
    Geometry& getGeometry()
    {
        return geometry_;
    }
    LearningAnnotation& getLearningAnnotation()
    {
        return learningAnnotation_;
    }

    void setGeometry(const Geometry& geometry)
    {
        geometry_ = geometry;
    }
    void setLearningAnnotation(const LearningAnnotation& learningAnnotation)
    {
        learningAnnotation_ = learningAnnotation;
    }

    const GroundTruth& getGroundTruth() const
    {
        return groundTruth_;
    }

    void setGroundTruth(const GroundTruth& groundTruth)
    {
        groundTruth_ = groundTruth;
    }

private:
    // meta data
    uint64_t frame_timestamp_;

    // identifiable data extracted from clusters
    Geometry geometry_;
    LearningAnnotation learningAnnotation_;
    GroundTruth groundTruth_;
    // add more:
    // - color
};

} /* namespace rs_log_learn */

#endif /* RS_LOG_LEARN_SRC_MPIDENTIFIABLE_H_ */
