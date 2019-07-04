#ifndef __PHYSOVERLAPFILTERCALLBACK_H__
#define __PHYSOVERLAPFILTERCALLBACK_H__

#include "common.hpp"

class PhysOverlapFilterCallback
	: public btOverlapFilterCallback
{
	virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const override;
};

#endif