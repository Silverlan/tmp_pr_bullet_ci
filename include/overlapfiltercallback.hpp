/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef __PHYSOVERLAPFILTERCALLBACK_H__
#define __PHYSOVERLAPFILTERCALLBACK_H__

#include "common.hpp"

class PhysOverlapFilterCallback
	: public btOverlapFilterCallback
{
	virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const override;
};

#endif