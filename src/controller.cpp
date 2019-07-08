#include "controller.hpp"
#include "environment.hpp"
#include "collision_object.hpp"
#include "shape.hpp"
#include "kinematic_character_controller.hpp"

#pragma optimize("",off)
pragma::physics::BtController::BtController(IEnvironment &env,const util::TSharedHandle<IGhostObject> &ghostObject,std::unique_ptr<PhysKinematicCharacterController> controller)
	: IController{env,util::shared_handle_cast<IGhostObject,ICollisionObject>(ghostObject)},m_controller{std::move(controller)}
{}
pragma::physics::BtEnvironment &pragma::physics::BtController::GetBtEnv() const {return static_cast<BtEnvironment&>(m_physEnv);}
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

void pragma::physics::BtController::SetFootPos(const Vector3 &footPos)
{
	Vector3 npos = pos;

	auto rot = ControllerPhysObj::GetOrientation();
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
}
Vector3 pragma::physics::BtController::GetFootPos() const
{
	Vector3 pos = ControllerPhysObj::GetPosition();

	auto rot = ControllerPhysObj::GetOrientation();
	auto offset = Vector3(0,-m_height *0.5f,0);
	uvec::rotate(&offset,rot);
	pos += offset;

	// Deprecated
	// pos.y -= m_height *0.5f;

	return pos;
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

pragma::physics::IController::CollisionFlags pragma::physics::BtController::DoMove(Vector3 &disp)
{
	m_controller->setWalkDirection(btVector3(disp.x,disp.y,disp.z) *BtEnvironment::WORLD_SCALE);
	return CollisionFlags::None;
}
#pragma optimize("",on)
