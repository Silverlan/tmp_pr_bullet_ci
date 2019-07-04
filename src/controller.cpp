#include "controller.hpp"
#include "environment.hpp"
#include "collision_object.hpp"
#include "shape.hpp"
#include "kinematic_character_controller.hpp"

#pragma optimize("",off)
pragma::physics::BtController::BtController(IEnvironment &env,IGhostObject &ghostObject,IConvexShape &shape,std::unique_ptr<PhysKinematicCharacterController> controller)
	: IController{env,ghostObject,shape},m_controller{std::move(controller)}
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
	world->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(dynamic_cast<BtGhostObject*>(GetGhostObject())->GetInternalObject().getBroadphaseHandle(),world->getDispatcher());
}

PhysKinematicCharacterController *pragma::physics::BtController::GetCharacterController() {return m_controller.get();}

void pragma::physics::BtController::SetWalkDirection(Vector3 &disp)
{
	m_controller->setWalkDirection(btVector3(disp.x,disp.y,disp.z) *BtEnvironment::WORLD_SCALE);
}

Vector3 pragma::physics::BtController::GetWalkDirection() const
{
	return uvec::create(m_controller->getWalkDirection() /BtEnvironment::WORLD_SCALE);
}
#pragma optimize("",on)
