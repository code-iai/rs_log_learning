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

private:
    // meta data
    uint64_t frame_timestamp_;

    // identifiable data extracted from clusters
    Geometry geometry_;
    LearningAnnotation learningAnnotation_;
    // add more:
    // - LearnAnnotation
    // - GroundTruth
};

} /* namespace rs_log_learn */

#endif /* RS_LOG_LEARN_SRC_MPIDENTIFIABLE_H_ */
