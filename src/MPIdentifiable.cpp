/*
 * MPIdentifiable.cpp
 *
 *  Created on: Feb 18, 2015
 *      Author: andre
 */

#include "MPIdentifiable.h"

namespace rs_log_learn
{

MPIdentifiable::MPIdentifiable(uint64_t frame_timestamp)
{
	this->frame_timestamp_ = frame_timestamp;
}

MPIdentifiable::~MPIdentifiable()
{
}

} /* namespace rs_log_learn */
