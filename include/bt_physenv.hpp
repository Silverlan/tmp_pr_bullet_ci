/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef __BT_PHYSENVIRONMENT_H__
#define __BT_PHYSENVIRONMENT_H__

#include "common.hpp"

/* Defined in bullet/src/BulletCollision/CollisionDispatch/btCollisionWorld.cpp */
struct btBridgedManifoldResultCustom : public btManifoldResult
{

	btCollisionWorld::ContactResultCallback&	m_resultCallback;

	btBridgedManifoldResultCustom( const btCollisionObjectWrapper* obj0Wrap,const btCollisionObjectWrapper* obj1Wrap,btCollisionWorld::ContactResultCallback& resultCallback );

	virtual void addContactPoint(const btVector3& normalOnBInWorld,const btVector3& pointInWorld,btScalar depth);

};
struct btSingleContactCallbackCustom : public btBroadphaseAabbCallback
{

	btCollisionObject* m_collisionObject;
	btCollisionWorld*	m_world;
	btCollisionWorld::ContactResultCallback&	m_resultCallback;


	btSingleContactCallbackCustom(btCollisionObject* collisionObject, btCollisionWorld* world,btCollisionWorld::ContactResultCallback& resultCallback);

	virtual bool	process(const btBroadphaseProxy* proxy);
};

#endif