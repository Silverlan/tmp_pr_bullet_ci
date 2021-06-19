/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "controller.hpp"
#include "environment.hpp"
#include "collision_object.hpp"
#include "shape.hpp"
#include "kinematic_character_controller.hpp"
#include <pragma/networkstate/networkstate.h>
#include <pragma/game/game.h>
#include <pragma/entities/components/base_character_component.hpp>
#include <BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.h>
#include <BulletCollision/NarrowPhaseCollision/btManifoldPoint.h>

Vector3 pragma::physics::PhysContactInfo::GetContactNormal(const Vector3 &n,int8_t controllerIndex)
{
	if(controllerIndex == 1)
		return -n;
	return n;
}

double pragma::physics::PhysContactInfo::CalcXZDistance(const btManifoldPoint &contactPoint,int8_t controllerIndex)
{
	auto &localPoint = (controllerIndex == 0) ? contactPoint.m_localPointA : contactPoint.m_localPointB;
	return umath::pow2(localPoint.x()) +umath::pow2(localPoint.z());
}

pragma::physics::PhysContactInfo::PhysContactInfo(const btManifoldPoint &contactPoint,int8_t controllerIndex)
	: contactPoint{contactPoint},controllerIndex{controllerIndex}
{}

Vector3 pragma::physics::PhysContactInfo::GetContactNormal() const
{
	return GetContactNormal(uvec::create(contactPoint.m_normalWorldOnB),controllerIndex);
}

double pragma::physics::PhysContactInfo::CalcXZDistance() const {return CalcXZDistance(contactPoint,controllerIndex);}

/////////////

pragma::physics::BtController::BtController(IEnvironment &env,const std::shared_ptr<BtConvexShape> &shape,const util::TSharedHandle<IGhostObject> &ghostObject,std::unique_ptr<PhysKinematicCharacterController> controller,const Vector3 &halfExtents,ShapeType shapeType)
	: IController{env,util::shared_handle_cast<IGhostObject,ICollisionObject>(ghostObject),halfExtents,shapeType},m_controller{std::move(controller)},m_shape{shape}
{}
pragma::physics::BtEnvironment &pragma::physics::BtController::GetBtEnv() const {return static_cast<BtEnvironment&>(m_physEnv);}
void pragma::physics::BtController::RemoveWorldObject()
{
	auto *world = GetBtEnv().GetWorld();
	world->removeCharacter(m_controller.get());
}
void pragma::physics::BtController::DoAddWorldObject()
{
	RemoveWorldObject();
	auto *world = GetBtEnv().GetWorld();
	world->addCharacter(m_controller.get());
}
Vector3 pragma::physics::BtController::GetDimensions() const
{
	auto *shape = GetShape();
	if(shape == nullptr)
		return Vector3(0.f,0.f,0.f);
	auto &btShape = static_cast<const btConvexInternalShape&>(dynamic_cast<const BtConvexShape&>(*shape).GetBtShape());
	auto dimensions = btShape.getImplicitShapeDimensions() /BtEnvironment::WORLD_SCALE;
	return Vector3(dimensions.x(),dimensions.y(),dimensions.z());
}
void pragma::physics::BtController::SetDimensions(const Vector3 &dimensions)
{
	auto *shape = GetShape();
	if(shape == nullptr)
		return;
	auto &btShape = static_cast<btConvexInternalShape&>(dynamic_cast<BtConvexShape&>(*shape).GetBtShape());
	btShape.setImplicitShapeDimensions(btVector3(dimensions.x,dimensions.y,dimensions.z) *BtEnvironment::WORLD_SCALE);
	auto *world = GetBtEnv().GetWorld();
	world->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(m_controller->getGhostObject()->getBroadphaseHandle(),world->getDispatcher());
}

pragma::physics::IController::CollisionFlags pragma::physics::BtController::GetCollisionFlags() const
{
	// TODO
	return CollisionFlags::None;
}
pragma::physics::IShape *pragma::physics::BtController::GetGroundShape() const
{
	if(!m_groundInfo.has_value())
		return nullptr;
	auto idx = m_groundInfo->contactInfo.controllerIndex;
	auto &contactPoint = m_groundInfo->contactInfo.contactPoint;
	auto &contactObject = (idx == 0) ? m_groundInfo->contactInfo.contactShape0 : m_groundInfo->contactInfo.contactShape1;
	return !contactObject.expired() ? contactObject.lock().get() : nullptr;
}
pragma::physics::IRigidBody *pragma::physics::BtController::GetGroundBody() const
{
	if(!m_groundInfo.has_value())
		return {};
	auto idx = m_groundInfo->contactInfo.controllerIndex;
	auto &contactPoint = m_groundInfo->contactInfo.contactPoint;
	auto &contactObject = (idx == 0) ? m_groundInfo->contactInfo.contactObject1 : m_groundInfo->contactInfo.contactObject0;
	auto *o = contactObject.Get();
	if(!o)
		return nullptr;
	return o->IsRigid() ? const_cast<IRigidBody*>(o->GetRigidBody()) : nullptr;
}
pragma::physics::IMaterial *pragma::physics::BtController::GetGroundMaterial() const
{
	auto *shape = GetGroundShape();
	return shape ? shape->GetMaterial() : nullptr;
}
bool pragma::physics::BtController::IsTouchingGround() const {return m_groundInfo.has_value();}
std::optional<Vector3> pragma::physics::BtController::GetGroundTouchPos() const
{
	if(!m_groundInfo.has_value())
		return {};
	auto idx = m_groundInfo->contactInfo.controllerIndex;
	auto &contactPoint = m_groundInfo->contactInfo.contactPoint;
	return BtEnvironment::ToPragmaPosition((idx == 0) ? contactPoint.getPositionWorldOnA() : contactPoint.getPositionWorldOnB());
}
std::optional<Vector3> pragma::physics::BtController::GetGroundTouchNormal() const
{
	if(!m_groundInfo.has_value())
		return {};
	auto idx = m_groundInfo->contactInfo.controllerIndex;
	auto &contactPoint = m_groundInfo->contactInfo.contactPoint;
	return PhysContactInfo::GetContactNormal(uvec::create(contactPoint.m_normalWorldOnB),idx);
}
void pragma::physics::BtController::SetPos(const Vector3 &pos)
{
	auto &transform = m_controller->getGhostObject()->getWorldTransform();
	transform.setOrigin(btVector3(pos.x,pos.y,pos.z) *static_cast<float>(BtEnvironment::WORLD_SCALE));
}
Vector3 pragma::physics::BtController::GetPos() const
{
	auto &transform = m_controller->getGhostObject()->getWorldTransform();
	auto &origin = transform.getOrigin();
	return Vector3(origin.x(),origin.y(),origin.z()) /static_cast<float>(BtEnvironment::WORLD_SCALE);
}
void pragma::physics::BtController::SetUpDirection(const Vector3 &up)
{
	m_controller->setUp(BtEnvironment::ToBtNormal(up));
}
Vector3 pragma::physics::BtController::GetUpDirection() const
{
	return BtEnvironment::ToPragmaNormal(m_controller->getUp());
}

void pragma::physics::BtController::SetSlopeLimit(umath::Degree slopeLimit)
{
	m_controller->setMaxSlope(umath::deg_to_rad(slopeLimit));
}
umath::Degree pragma::physics::BtController::GetSlopeLimit() const
{
	return umath::rad_to_deg(m_controller->getMaxSlope());
}

void pragma::physics::BtController::SetStepHeight(float stepHeight)
{
	m_controller->setStepHeight(BtEnvironment::ToBtDistance(stepHeight));
}
float pragma::physics::BtController::GetStepHeight() const
{
	return BtEnvironment::ToPragmaDistance(m_controller->getStepHeight());
}

Vector3 pragma::physics::BtController::GetLinearVelocity() const {return m_velocity;}
void pragma::physics::BtController::SetLinearVelocity(const Vector3 &vel) {m_velocity = vel;}

void pragma::physics::BtController::PreSimulate(float dt)
{
	auto &t = m_controller->getGhostObject()->getWorldTransform();
	m_preSimulationPosition = BtEnvironment::ToPragmaPosition(t.getOrigin());
	auto vel = GetMoveVelocity();
	ClearGroundContactPoint();
	DoMove(vel);
}

void pragma::physics::BtController::PostSimulate(float dt)
{
	if(dt > 0.f)
	{
		auto &t = m_controller->getGhostObject()->getWorldTransform();
		auto deltaPos = BtEnvironment::ToPragmaPosition(t.getOrigin()) -m_preSimulationPosition;
		m_velocity = deltaPos *(1.f /dt);
	}
}

void pragma::physics::BtController::SetFootPos(const Vector3 &footPos)
{
	auto origin = footPos;
	if(m_shape)
	{
		auto &shape = m_shape->GetBtConvexShape();
		auto type = shape.getShapeType();
		switch(type)
		{
		case BroadphaseNativeTypes::CAPSULE_SHAPE_PROXYTYPE:
			origin += GetUpDirection() *GetHalfExtents().y;
			break;
		case BroadphaseNativeTypes::BOX_SHAPE_PROXYTYPE:
			// static_cast<btBoxShape&>(shape).getBoundingSphere();
			break;
		}
	}
	auto t = m_controller->getGhostObject()->getWorldTransform();
	t.setOrigin(BtEnvironment::ToBtPosition(origin));
	m_controller->getGhostObject()->setWorldTransform(t);
	// TODO
#if 0
	Vector3 npos = footPos;

	auto rot = IController::GetOrientation();
	auto offset = Vector3(0,-m_height *0.5f,0);
	uvec::rotate(&offset,rot);
	npos -= offset;

	// Deprecated
	// npos.y += m_height *0.5f;

	ControllerPhysObj::SetPosition(npos);
	//ControllerPhysObj::SetPosition(pos);

	BOX CONTROLLER:
	auto t = m_collisionObject->GetWorldTransform();
	Vector3 posCur = t.GetOrigin();
	PhysObj::SetPosition(pos);
	m_posLast += pos -posCur;
#endif
}
Vector3 pragma::physics::BtController::GetFootPos() const
{
	auto &t = m_controller->getGhostObject()->getWorldTransform();
	auto origin = BtEnvironment::ToPragmaPosition(t.getOrigin());
	if(m_shape)
	{
		auto &shape = m_shape->GetBtConvexShape();
		auto type = shape.getShapeType();
		switch(type)
		{
		case BroadphaseNativeTypes::CAPSULE_SHAPE_PROXYTYPE:
			origin -= GetUpDirection() *GetHalfExtents().y;
			break;
		case BroadphaseNativeTypes::BOX_SHAPE_PROXYTYPE:
			// static_cast<btBoxShape&>(shape).getBoundingSphere();
			break;
		}
	}
	return origin;
	// TODO
	/*Vector3 pos = ControllerPhysObj::GetPosition();

	auto rot = ControllerPhysObj::GetOrientation();
	auto offset = Vector3(0,-m_height *0.5f,0);
	uvec::rotate(&offset,rot);
	pos += offset;

	// Deprecated
	// pos.y -= m_height *0.5f;
	*/
	//return ControllerPhysObj::GetPosition();
}

void pragma::physics::BtController::Resize(float newHeight)
{
	auto dimensions = GetDimensions();
	dimensions.y = newHeight;
	SetDimensions(dimensions);
	// TODO: Update foot position
}

PhysKinematicCharacterController *pragma::physics::BtController::GetCharacterController() {return m_controller.get();}

const pragma::physics::BtConvexShape *pragma::physics::BtController::GetShape() const {return const_cast<BtController*>(this)->GetShape();}
pragma::physics::BtConvexShape *pragma::physics::BtController::GetShape() {return m_shape.get();}

void pragma::physics::BtController::DoMove(Vector3 &disp)
{
	m_controller->setWalkDirection(btVector3(disp.x,disp.y,disp.z) *BtEnvironment::WORLD_SCALE);
}

void pragma::physics::BtController::ClearGroundContactPoint()
{
	m_groundInfo = {};
	// SetCurrentFriction(1.f); // TODO
}

bool pragma::physics::BtController::SetGroundContactPoint(const btManifoldPoint &contactPoint,int32_t idx,const btCollisionObject *o,const btCollisionObject *oOther)
{
	auto d = PhysContactInfo::CalcXZDistance(contactPoint,idx);
	if(m_groundInfo.has_value())
		m_groundInfo->minContactDistance = umath::min(m_groundInfo->contactDistance,d);

	// Check if ground is walkable
	auto n = PhysContactInfo::GetContactNormal(uvec::create(contactPoint.m_normalWorldOnB),idx);
	auto angle = umath::acos(uvec::dot(n,GetUpDirection()));
	auto slopeLimit = GetSlopeLimit();
	auto bGroundWalkable = (angle <= umath::deg_to_rad(slopeLimit));

	auto pos = uvec::create(((idx == 0) ? contactPoint.getPositionWorldOnA() : contactPoint.getPositionWorldOnB()) /BtEnvironment::WORLD_SCALE);
	//GetOwner()->GetEntity().GetNetworkState()->GetGameState()->DrawLine(pos,pos +n *100.f,Color::Red,0.3f);
	if(bGroundWalkable == false)
	{
		// Discard this point if we already have a walkable point (even if this point is closer)
		if(m_groundInfo.has_value() && m_groundInfo->groundWalkable == true)
			return false;
	}

	if(m_groundInfo.has_value() == true && d >= m_groundInfo->contactDistance)
		return false; // We already have a better candidate, discard this one

	auto dCur = m_groundInfo.has_value() ? m_groundInfo->contactDistance : std::numeric_limits<double>::max();
	m_groundInfo = {GroundInfo{contactPoint,static_cast<int8_t>(idx)}};
	m_groundInfo->groundWalkable = bGroundWalkable;
	m_groundInfo->contactDistance = d;
	m_groundInfo->minContactDistance = umath::min(d,dCur);
	auto &contactInfo = m_groundInfo->contactInfo;

	auto *obj0 = static_cast<pragma::physics::ICollisionObject*>(o->getUserPointer());
	auto *obj1 = static_cast<pragma::physics::ICollisionObject*>(oOther->getUserPointer());
	contactInfo.contactObject0 = (obj0 != nullptr) ? util::weak_shared_handle_cast<IBase,ICollisionObject>(obj0->GetHandle()) : util::TWeakSharedHandle<ICollisionObject>{};
	contactInfo.contactObject1 = (obj1 != nullptr) ? util::weak_shared_handle_cast<IBase,ICollisionObject>(obj1->GetHandle()) : util::TWeakSharedHandle<ICollisionObject>{};

	auto fGetShape = [](pragma::physics::ICollisionObject &o,int shapeIdx) -> pragma::physics::IShape* {
		auto *shape = o.GetCollisionShape();
		if(!shape)
			return nullptr;
		auto *compoundShape = shape->GetCompoundShape();
		if(!compoundShape)
			return shape;
		auto &shapes = compoundShape->GetShapes();
		if(shapeIdx < 0 || shapeIdx >= shapes.size())
			return nullptr;
		return shapes[shapeIdx].shape.get();
	};
	contactInfo.contactShape0 = {};
	contactInfo.contactShape1 = {};
	if(contactInfo.contactObject0.IsValid())
	{
		auto *shape = fGetShape(*contactInfo.contactObject0,contactPoint.m_index0);
		contactInfo.contactShape0 = shape ? std::static_pointer_cast<IShape>(shape->shared_from_this()) : nullptr;
	}
	if(contactInfo.contactObject1.IsValid())
	{
		auto *shape = fGetShape(*contactInfo.contactObject1,contactPoint.m_index1);
		contactInfo.contactShape1 = shape ? std::static_pointer_cast<IShape>(shape->shared_from_this()) : nullptr;
	}

	// TODO
	//SetCurrentFriction(CFloat(contactPoint.m_combinedFriction));
	auto *oSurface = (idx == 0) ? oOther : o;
	auto *shape = oSurface->getCollisionShape();
	if(shape->getShapeType() == MULTIMATERIAL_TRIANGLE_MESH_PROXYTYPE)
	{
		throw std::runtime_error{"Not implemented!"};
		// TODO
		/*auto *mtShape = const_cast<btMultimaterialTriangleMeshShape*>(static_cast<const btMultimaterialTriangleMeshShape*>(shape));
		auto *matProps = mtShape->getMaterialProperties((idx == 0) ? contactPoint.m_partId1 : contactPoint.m_partId0,(idx == 0) ? contactPoint.m_index1 : contactPoint.m_index0);
		auto *surface = static_cast<const SurfaceMaterial*>(mtShape->getMaterialProperties((idx == 0) ? contactPoint.m_partId1 : contactPoint.m_partId0,(idx == 0) ? contactPoint.m_index1 : contactPoint.m_index0));
		if(surface != nullptr)
			contactInfo.surfaceMaterialId = CUInt32(surface->GetIndex());*/
	}
	else
	{
		auto *obj = static_cast<pragma::physics::ICollisionObject*>(oSurface->getUserPointer());
		if(obj != nullptr)
		{
			auto *surface = GetBtEnv().GetNetworkState().GetGameState()->GetSurfaceMaterial(obj->GetSurfaceMaterial());
			if(surface != nullptr)
				contactInfo.surfaceMaterialId = CUInt32(surface->GetIndex());
		}
	}
	return true;
}
