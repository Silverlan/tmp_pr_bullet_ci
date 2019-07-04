#ifndef __PR_BT_COMMON_HPP__
#define __PR_BT_COMMON_HPP__

#pragma warning(disable: 4127)
#pragma warning(disable: 4100)
#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#pragma warning(default: 4100)
#pragma warning(default: 4127)
#include <mathutil/uvec.h>

namespace uvec
{
	btVector3 create_bt(const Vector3 &v);
	Vector3 create(const btVector3 &v);
};

namespace uquat
{
	btQuaternion create_bt(const Quat &v);
	Quat create(const btQuaternion &v);
};

#endif
