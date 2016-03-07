/*
 * MPIdentifiable.cpp
 *
 *  Created on: Feb 18, 2015
 *      Author: andre
 */

#include <rs_log_learning/MPIdentifiable.h>

namespace rs_log_learning
{

MPIdentifiable::MPIdentifiable(uint64_t frame_timestamp)
{
  this->frame_timestamp_ = frame_timestamp;
}

MPIdentifiable::~MPIdentifiable()
{
}

} /* namespace rs_log_learning */
