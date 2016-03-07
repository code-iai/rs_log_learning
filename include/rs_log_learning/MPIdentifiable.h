/*
 * MPIdentifiable.h
 *
 *  Created on: Feb 18, 2015
 *      Author: andre
 */

#ifndef RS_LOG_LEARN_SRC_MPIDENTIFIABLE_H_
#define RS_LOG_LEARN_SRC_MPIDENTIFIABLE_H_

// MP includes
#include <rs_log_learning/containers/Geometry.h>
#include <rs_log_learning/containers/LearningAnnotation.h>
#include <rs_log_learning/containers/SemanticColor.h>
#include <rs_log_learning/containers/GroundTruth.h>

namespace rs_log_learning
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
  Geometry &getGeometry()
  {
    return geometry_;
  }
  LearningAnnotation &getLearningAnnotation()
  {
    return learningAnnotation_;
  }

  const GroundTruth &getGroundTruth() const
  {
    return groundTruth_;
  }

  SemanticColor &getSemColor()
  {
    return semColor_;
  }

  void setGeometry(const Geometry &geometry)
  {
    geometry_ = geometry;
  }
  void setLearningAnnotation(const LearningAnnotation &learningAnnotation)
  {
    learningAnnotation_ = learningAnnotation;
  }

  void setGroundTruth(const GroundTruth &groundTruth)
  {
    groundTruth_ = groundTruth;
  }

  void setSemColor(const SemanticColor &semColor)
  {
    semColor_ = semColor;
  }

private:
  // meta data
  uint64_t frame_timestamp_;

  // identifiable data extracted from clusters
  Geometry geometry_;
  SemanticColor semColor_;
  LearningAnnotation learningAnnotation_;
  GroundTruth groundTruth_;
};

} /* namespace rs_log_learning */

#endif /* RS_LOG_LEARN_SRC_MPIDENTIFIABLE_H_ */
