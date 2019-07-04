#include "collision_object.hpp"
#include "environment.hpp"
#include "shape.hpp"
#include "motion_state.hpp"
#include "common.hpp"
#include <pragma/networkstate/networkstate.h>
#include <pragma/game/game.h>
#include <pragma/model/modelmesh.h>

#pragma optimize("",off)
class SimpleMotionState
	: public btMotionState
{
public:
	SimpleMotionState()=default;
	// TODO: Implement this
	virtual void getWorldTransform(btTransform &worldTrans) const override {}
	virtual void setWorldTransform(const btTransform &worldTrans) override {}
};

pragma::physics::BtCollisionObject::BtCollisionObject(IEnvironment &env,std::unique_ptr<btCollisionObject> o,IShape &shape)
	: ICollisionObject{env,shape},m_collisionObject{std::move(o)}
{
	GetInternalObject().setUserPointer(static_cast<ICollisionObject*>(this));
}

void pragma::physics::BtCollisionObject::Initialize()
{
	ICollisionObject::Initialize();
	UpdateCCD();
}
btCollisionObject &pragma::physics::BtCollisionObject::GetBtCollisionObject() {return *m_collisionObject;}
pragma::physics::BtRigidBody *pragma::physics::BtCollisionObject::GetBtRigidBody() {return nullptr;}
const pragma::physics::BtRigidBody *pragma::physics::BtCollisionObject::GetBtRigidBody() const {return const_cast<BtCollisionObject*>(this)->GetBtRigidBody();}
pragma::physics::BtSoftBody *pragma::physics::BtCollisionObject::GetBtSoftBody() {return nullptr;}
const pragma::physics::BtSoftBody *pragma::physics::BtCollisionObject::GetBtSoftBody() const {return const_cast<BtCollisionObject*>(this)->GetBtSoftBody();}
pragma::physics::BtGhostObject *pragma::physics::BtCollisionObject::GetBtGhostObject() {return nullptr;}
const pragma::physics::BtGhostObject *pragma::physics::BtCollisionObject::GetBtGhostObject() const {return const_cast<BtCollisionObject*>(this)->GetBtGhostObject();}

void pragma::physics::BtCollisionObject::Spawn()
{
	pragma::physics::ICollisionObject::Spawn();
	auto *broadphase = m_collisionObject->getBroadphaseHandle();
	if(broadphase == nullptr)
		return;
	broadphase->m_collisionFilterMask = static_cast<int16_t>(umath::to_integral(m_collisionFilterMask));
	broadphase->m_collisionFilterGroup = static_cast<int16_t>(umath::to_integral(m_collisionFilterGroup));
	UpdateCCD();
}

void pragma::physics::BtCollisionObject::PutToSleep()
{
	SetActivationState(ActivationState::WaitForDeactivation);
}

void pragma::physics::BtCollisionObject::SetSimulationEnabled(bool b)
{
	if(b == IsSimulationEnabled())
		return;
	m_simulationEnabled.first = b;
	if(b == false)
	{
		m_collisionObject->setActivationState(DISABLE_SIMULATION);
		m_collisionObject->setCollisionFlags(m_collisionObject->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
		m_simulationEnabled.second = GetCollisionFilterGroup();
		SetCollisionFilterGroup(CollisionMask::NoCollision);
	}
	else
	{
		m_collisionObject->setCollisionFlags(m_collisionObject->getCollisionFlags() &~btCollisionObject::CF_NO_CONTACT_RESPONSE);
		SetCollisionFilterGroup(m_simulationEnabled.second);
		m_collisionObject->forceActivationState(ACTIVE_TAG);
		m_collisionObject->activate();
	}
}
bool pragma::physics::BtCollisionObject::IsSimulationEnabled() const {return m_simulationEnabled.first;}
void pragma::physics::BtCollisionObject::SetCollisionsEnabled(bool enabled)
{
	if(enabled == false)
		m_collisionObject->setCollisionFlags(m_collisionObject->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
	else
		m_collisionObject->setCollisionFlags(m_collisionObject->getCollisionFlags() & ~btCollisionObject::CF_NO_CONTACT_RESPONSE);
}

void pragma::physics::BtCollisionObject::SetCCDEnabled(bool b)
{
	m_bCcdEnabled = b;
	UpdateCCD();
}

void pragma::physics::BtCollisionObject::UpdateCCD()
{
	// Dirty hack: CCD does not work with custom collision rules, so disable it if there are any
	if(m_bCcdEnabled == false || m_collisionFilterGroup != CollisionMask::Default || m_collisionFilterMask != CollisionMask::Default)
	{
		m_collisionObject->setCcdMotionThreshold(0.0);
		m_collisionObject->setCcdSweptSphereRadius(0.0);
	}
	else
	{
		m_collisionObject->setCcdMotionThreshold(BtEnvironment::CCD_MOTION_THRESHOLD);
		m_collisionObject->setCcdSweptSphereRadius(BtEnvironment::CCD_SWEPT_SPHERE_RADIUS);
	}
}

void pragma::physics::BtCollisionObject::GetAABB(Vector3 &min,Vector3 &max) const
{
	if(m_shape == nullptr)
	{
		min = {};
		max = {};
		return;
	}
	m_shape->GetAABB(min,max);
}

bool pragma::physics::BtCollisionObject::IsTrigger()
{
	int flags = m_collisionObject->getCollisionFlags();
	return (flags &btCollisionObject::CF_NO_CONTACT_RESPONSE) != 0 ? true : false;
}

bool pragma::physics::BtCollisionObject::IsAsleep() const
{
	auto state = m_collisionObject->getActivationState();
	return (state == DISABLE_SIMULATION || state == ISLAND_SLEEPING) ? true : false;
}

pragma::physics::Transform pragma::physics::BtCollisionObject::GetWorldTransform()
{
	return BtEnvironment::CreateTransform(m_collisionObject->getWorldTransform());
}

void pragma::physics::BtCollisionObject::SetWorldTransform(const pragma::physics::Transform &t)
{
	m_collisionObject->setWorldTransform(BtEnvironment::CreateBtTransform(t));
}

Vector3 pragma::physics::BtCollisionObject::GetPos() const
{
	btTransform &t = m_collisionObject->getWorldTransform();
	btVector3 &p = t.getOrigin();
	return Vector3(p.x() /BtEnvironment::WORLD_SCALE,p.y() /BtEnvironment::WORLD_SCALE,p.z() /BtEnvironment::WORLD_SCALE);
}
void pragma::physics::BtCollisionObject::SetPos(const Vector3 &pos)
{
	btTransform &t = m_collisionObject->getWorldTransform();
	btVector3 &p = t.getOrigin();
	p.setX(pos.x *BtEnvironment::WORLD_SCALE);
	p.setY(pos.y *BtEnvironment::WORLD_SCALE);
	p.setZ(pos.z *BtEnvironment::WORLD_SCALE);

	UpdateAABB();
}

Quat pragma::physics::BtCollisionObject::GetRotation() const
{
	btTransform &t = m_collisionObject->getWorldTransform();
	btQuaternion rot = t.getRotation();
	return Quat(
		static_cast<float>(rot.w()),
		static_cast<float>(rot.x()),
		static_cast<float>(rot.y()),
		static_cast<float>(rot.z())
	);
}

void pragma::physics::BtCollisionObject::SetRotation(const Quat &rot)
{
	btTransform &t = m_collisionObject->getWorldTransform();
	t.setRotation(btQuaternion(rot.x,rot.y,rot.z,rot.w));
}

int pragma::physics::BtCollisionObject::GetCollisionFlags() const {return m_collisionObject->getCollisionFlags();}

void pragma::physics::BtCollisionObject::SetCollisionFlags(int flags) {return m_collisionObject->setCollisionFlags(flags);}

void pragma::physics::BtCollisionObject::ApplyCollisionShape(pragma::physics::IShape *optShape)
{
	auto *btShape = dynamic_cast<BtShape*>(optShape);
	if(optShape != nullptr)
	{
		auto &pShape = btShape->GetBtShape();
		m_collisionObject->setCollisionShape(&pShape);
		UpdateSurfaceMaterial();

		if(optShape->IsTrigger())
			m_collisionObject->setCollisionFlags(m_collisionObject->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
	}
	else
		m_collisionObject->setCollisionShape(nullptr);
}

void pragma::physics::BtCollisionObject::DoSetCollisionFilterGroup(CollisionMask group)
{
	auto *broadphase = m_collisionObject->getBroadphaseHandle();
	if(broadphase == nullptr)
		return;
	broadphase->m_collisionFilterGroup = static_cast<int16_t>(umath::to_integral(group));
}

void pragma::physics::BtCollisionObject::DoSetCollisionFilterMask(CollisionMask mask)
{
	auto *broadphase = m_collisionObject->getBroadphaseHandle();
	if(broadphase == nullptr)
		return;
	broadphase->m_collisionFilterMask = static_cast<int16_t>(umath::to_integral(mask));
}

void pragma::physics::BtCollisionObject::WakeUp(bool forceActivation) {m_collisionObject->activate(forceActivation);}

pragma::physics::BtCollisionObject::ActivationState pragma::physics::BtCollisionObject::GetActivationState() const
{
	auto state = ActivationState::Active;
	switch(m_collisionObject->getActivationState())
	{
	case ACTIVE_TAG:
		state = ActivationState::Active;
		break;
	case DISABLE_DEACTIVATION:
		state = ActivationState::AlwaysActive;
		break;
	case DISABLE_SIMULATION:
		state = ActivationState::AlwaysInactive;
		break;
	case ISLAND_SLEEPING:
		state = ActivationState::Asleep;
		break;
	case WANTS_DEACTIVATION:
		state = ActivationState::WaitForDeactivation;
		break;
	}
	static_assert(umath::to_integral(ActivationState::Count) == 5);
	return state;
}
void pragma::physics::BtCollisionObject::SetActivationState(ActivationState state)
{
	int btState = 0;
	switch(state)
	{
	case ActivationState::Active:
		btState = ACTIVE_TAG;
		break;
	case ActivationState::AlwaysActive:
		btState = DISABLE_DEACTIVATION;
		break;
	case ActivationState::AlwaysInactive:
		btState = DISABLE_SIMULATION;
		break;
	case ActivationState::Asleep:
		btState = ISLAND_SLEEPING;
		break;
	case ActivationState::WaitForDeactivation:
		btState = WANTS_DEACTIVATION;
		break;
	}
	static_assert(umath::to_integral(ActivationState::Count) == 5);
	m_collisionObject->setActivationState(btState);
}

void pragma::physics::BtCollisionObject::SetContactProcessingThreshold(float threshold)
{
	m_collisionObject->setContactProcessingThreshold(threshold);
}

pragma::physics::BtEnvironment &pragma::physics::BtCollisionObject::GetBtEnv() const {return static_cast<BtEnvironment&>(m_physEnv);}
btCollisionObject &pragma::physics::BtCollisionObject::GetInternalObject() const {return *m_collisionObject;}

void pragma::physics::BtCollisionObject::DoAddWorldObject()
{
	auto *world = GetBtEnv().GetWorld();
	world->addCollisionObject(m_collisionObject.get(),umath::to_integral(m_collisionFilterGroup),umath::to_integral(m_collisionFilterMask));
}

void pragma::physics::BtCollisionObject::RemoveWorldObject()
{
	auto *world = GetBtEnv().GetWorld();
	world->removeCollisionObject(m_collisionObject.get());
}

bool pragma::physics::BtCollisionObject::IsStatic() const {return false;}
void pragma::physics::BtCollisionObject::SetStatic(bool b) {}

//////////////////////

pragma::physics::BtRigidBody::BtRigidBody(IEnvironment &env,std::unique_ptr<btRigidBody> body,float mass,IShape &shape,const Vector3 &localInertia)
	: ICollisionObject{env,shape},BtCollisionObject{env,std::move(body),shape},IRigidBody{env,mass,shape,localInertia}
{
	SetMassProps(mass,localInertia);
	m_motionState = std::make_unique<SimpleMotionState>();
	GetInternalObject().setMotionState(m_motionState.get());
}
pragma::physics::BtRigidBody::BtRigidBody(IEnvironment &env,float mass,BtShape &shape,const Vector3 &localInertia)
	: BtRigidBody(env,std::make_unique<btRigidBody>(mass,nullptr,&shape.GetBtShape(),btVector3(localInertia.x,localInertia.y,localInertia.z) *BtEnvironment::WORLD_SCALE),mass,shape,localInertia)
{}
pragma::physics::BtRigidBody::~BtRigidBody() {}
btRigidBody &pragma::physics::BtRigidBody::GetInternalObject() const {return static_cast<btRigidBody&>(BtCollisionObject::GetInternalObject());}
pragma::physics::BtRigidBody *pragma::physics::BtRigidBody::GetBtRigidBody() {return this;}
void pragma::physics::BtRigidBody::SetLinearDamping(float damping)
{
	auto &body = GetInternalObject();
	body.setDamping(damping,body.getAngularDamping());
}
void pragma::physics::BtRigidBody::SetAngularDamping(float damping)
{
	auto &body = GetInternalObject();
	body.setDamping(body.getLinearDamping(),damping);
}
float pragma::physics::BtRigidBody::GetLinearDamping() const
{
	auto &body = GetInternalObject();
	return CFloat(body.getLinearDamping());
}
float pragma::physics::BtRigidBody::GetAngularDamping() const
{
	auto &body = GetInternalObject();
	return CFloat(body.getAngularDamping());
}
void pragma::physics::BtRigidBody::SetLinearSleepingThreshold(float threshold) {SetSleepingThresholds(threshold,GetAngularSleepingThreshold());}
void pragma::physics::BtRigidBody::SetAngularSleepingThreshold(float threshold) {SetSleepingThresholds(GetLinearSleepingThreshold(),threshold);}
float pragma::physics::BtRigidBody::GetLinearSleepingThreshold() const
{
	auto &body = GetInternalObject();
	return body.getLinearSleepingThreshold();
}
float pragma::physics::BtRigidBody::GetAngularSleepingThreshold() const
{
	auto &body = GetInternalObject();
	return body.getAngularSleepingThreshold();
}
void pragma::physics::BtRigidBody::ClearForces()
{
	auto &body = GetInternalObject();
	return body.clearForces();
}
Vector3 pragma::physics::BtRigidBody::GetTotalForce()
{
	auto &body = GetInternalObject();
	auto force = body.getTotalForce() /BtEnvironment::WORLD_SCALE;
	return Vector3(force.x(),force.y(),force.z());
}
Vector3 pragma::physics::BtRigidBody::GetTotalTorque()
{
	auto &body = GetInternalObject();
	auto force = body.getTotalTorque() /BtEnvironment::WORLD_SCALE_SQR;
	return Vector3(force.x(),force.y(),force.z());
}

void pragma::physics::BtRigidBody::DoAddWorldObject()
{
	if(m_bSpawned == false)
		return;
	RemoveWorldObject();
	auto *world = GetBtEnv().GetWorld();
	world->addRigidBody(&GetInternalObject(),umath::to_integral(m_collisionFilterGroup),umath::to_integral(m_collisionFilterMask));
}

void pragma::physics::BtRigidBody::RemoveWorldObject()
{
	auto *world = GetBtEnv().GetWorld();
	world->removeRigidBody(&GetInternalObject());
}

bool pragma::physics::BtRigidBody::IsStatic() const {return GetMass() == 0.f && IsKinematic() == false;}

void pragma::physics::BtRigidBody::SetStatic(bool b)
{
	int flags = GetInternalObject().getCollisionFlags();
	if(b == true)
		flags |= btCollisionObject::CF_STATIC_OBJECT;
	else
		flags &= ~btCollisionObject::CF_STATIC_OBJECT;
	GetInternalObject().setCollisionFlags(flags);
	SetMass(b ? 0.f : m_mass);
}

void pragma::physics::BtRigidBody::ApplyForce(const Vector3 &force)
{
	if(IsStatic())
		return;
	GetInternalObject().activate(true);
	GetInternalObject().applyCentralForce(btVector3(force.x,force.y,force.z) *BtEnvironment::WORLD_SCALE *GetBtEnv().GetTimeScale());
}
void pragma::physics::BtRigidBody::ApplyForce(const Vector3 &force,const Vector3 &relPos)
{
	if(IsStatic())
		return;
	GetInternalObject().activate(true);
	auto btRelPos = uvec::create_bt(relPos) *BtEnvironment::WORLD_SCALE -GetInternalObject().getCenterOfMassPosition();
	GetInternalObject().applyForce(btVector3(force.x,force.y,force.z) *BtEnvironment::WORLD_SCALE *m_physEnv.GetTimeScale(),btRelPos);
}
void pragma::physics::BtRigidBody::ApplyImpulse(const Vector3 &impulse)
{
	if(IsStatic())
		return;
	GetInternalObject().activate(true);
	GetInternalObject().applyCentralImpulse(btVector3(impulse.x,impulse.y,impulse.z) *BtEnvironment::WORLD_SCALE);
}
void pragma::physics::BtRigidBody::ApplyImpulse(const Vector3 &impulse,const Vector3 &relPos)
{
	if(IsStatic())
		return;
	GetInternalObject().activate(true);
	auto btRelPos = uvec::create_bt(relPos) *BtEnvironment::WORLD_SCALE -GetInternalObject().getCenterOfMassPosition();
	GetInternalObject().applyImpulse(btVector3(impulse.x,impulse.y,impulse.z) *BtEnvironment::WORLD_SCALE,btRelPos);
}
void pragma::physics::BtRigidBody::ApplyTorque(const Vector3 &torque)
{
	if(IsStatic())
		return;
	GetInternalObject().activate(true);
	auto ts = m_physEnv.GetTimeScale();
	GetInternalObject().applyTorque(btVector3(torque.x,torque.y,torque.z) *BtEnvironment::WORLD_SCALE_SQR *(ts *ts));
}
void pragma::physics::BtRigidBody::ApplyTorqueImpulse(const Vector3 &torque)
{
	if(IsStatic())
		return;
	GetInternalObject().activate(true);
	GetInternalObject().applyTorqueImpulse(btVector3(torque.x,torque.y,torque.z) *BtEnvironment::WORLD_SCALE_SQR);
}

void pragma::physics::BtRigidBody::SetMassProps(float mass,const Vector3 &inertia)
{
	GetInternalObject().setMassProps(mass,btVector3(inertia.x,inertia.y,inertia.z) *BtEnvironment::WORLD_SCALE);
	m_mass = mass;
	m_inertia = inertia;
	SetCollisionFlags(GetCollisionFlags() &~btCollisionObject::CF_STATIC_OBJECT);

	// BUG: If CF_STATIC_OBJECT is set, soft-body objects will not collide with static rigid bodies! (Cause unknown)
	if(mass != 0.f)
		SetCollisionFlags(GetCollisionFlags() &~btCollisionObject::CF_STATIC_OBJECT);
	else
		SetCollisionFlags(GetCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
}

void pragma::physics::BtRigidBody::SetKinematic(bool bKinematic)
{
	if(bKinematic == IsKinematic())
		return;
	auto flags = GetCollisionFlags();
	if(bKinematic)
		flags |= btCollisionObject::CF_KINEMATIC_OBJECT;
	else
		flags &= ~btCollisionObject::CF_KINEMATIC_OBJECT;
	SetCollisionFlags(flags);

	if(bKinematic == false)
		m_motionState = std::make_unique<SimpleMotionState>();
	else
		m_motionState = std::make_unique<KinematicMotionState>(BtEnvironment::CreateTransform(GetInternalObject().getWorldTransform()));
	GetInternalObject().setMotionState(m_motionState.get());

	m_kinematicData = {};
}
bool pragma::physics::BtRigidBody::IsKinematic() const
{
	return (GetCollisionFlags() &btCollisionObject::CF_KINEMATIC_OBJECT) != 0;
}
Vector3 pragma::physics::BtRigidBody::GetLinearVelocity() const
{
	if(IsStatic() == true)
		return {};
	if(IsKinematic())
		return m_kinematicData.linearVelocity;
	const btVector3 &linVel = GetInternalObject().getLinearVelocity();
	return Vector3(linVel.x() /BtEnvironment::WORLD_SCALE,linVel.y() /BtEnvironment::WORLD_SCALE,linVel.z() /BtEnvironment::WORLD_SCALE);
}
Vector3 pragma::physics::BtRigidBody::GetAngularVelocity() const
{
	if(IsStatic() == true)
		return {};
	if(IsKinematic())
		return m_kinematicData.angularVelocity;
	const btVector3 &angVel = GetInternalObject().getAngularVelocity();
	return Vector3(angVel.x(),angVel.y(),angVel.z());
}

void pragma::physics::BtRigidBody::SetLinearVelocity(const Vector3 &vel)
{
	if(IsStatic() == true)
	{
		GetInternalObject().setLinearVelocity(btVector3{0.f,0.f,0.f}); // Can't apply velocity to static objects
		return;
	}
	GetInternalObject().activate(true);
	GetInternalObject().setLinearVelocity(btVector3(vel.x,vel.y,vel.z) *BtEnvironment::WORLD_SCALE);
	if(IsKinematic())
		m_kinematicData.linearVelocity = vel;
}
void pragma::physics::BtRigidBody::SetAngularVelocity(const Vector3 &vel)
{
	if(IsStatic() == true)
	{
		GetInternalObject().setAngularVelocity(btVector3{0.f,0.f,0.f}); // Can't apply angular velocity to static objects
		return;
	}
	GetInternalObject().activate(true);
	GetInternalObject().setAngularVelocity(btVector3(vel.x,vel.y,vel.z));
	if(IsKinematic())
		m_kinematicData.angularVelocity = vel;
}
float pragma::physics::BtRigidBody::GetMass() const {return m_mass;}
void pragma::physics::BtRigidBody::SetMass(float mass)
{
	Vector3 localInertia(0.f,0.f,0.f);
	m_shape->CalculateLocalInertia(mass,&localInertia);
	SetMassProps(mass,localInertia);
	if(mass == 0.f)
	{
		SetLinearVelocity({});
		SetAngularVelocity({});
	}
}
Vector3 &pragma::physics::BtRigidBody::GetInertia() {return m_inertia;}
Mat3 pragma::physics::BtRigidBody::GetInvInertiaTensorWorld() const
{
	GetInternalObject().updateInertiaTensor();
	auto &inertia = GetInternalObject().getInvInertiaTensorWorld();
	return Mat3{
		inertia[0][0],inertia[0][1],inertia[0][2],
		inertia[1][0],inertia[1][1],inertia[1][2],
		inertia[2][0],inertia[2][1],inertia[2][2],
	} *static_cast<float>(BtEnvironment::WORLD_SCALE_SQR);
}
void pragma::physics::BtRigidBody::SetInertia(const Vector3 &inertia) {SetMassProps(m_mass,inertia);}

void pragma::physics::BtRigidBody::SetLinearFactor(const Vector3 &factor) {GetInternalObject().setLinearFactor(btVector3(factor.x,factor.y,factor.z));}
void pragma::physics::BtRigidBody::SetAngularFactor(const Vector3 &factor) {GetInternalObject().setAngularFactor(btVector3(factor.x,factor.y,factor.z));}
Vector3 pragma::physics::BtRigidBody::GetLinearFactor() const
{
	auto &factor = GetInternalObject().getLinearFactor();
	return Vector3(factor.x(),factor.y(),factor.z());
}
Vector3 pragma::physics::BtRigidBody::GetAngularFactor() const
{
	auto &factor = GetInternalObject().getAngularFactor();
	return Vector3(factor.x(),factor.y(),factor.z());
}
Vector3 pragma::physics::BtRigidBody::GetPos() const
{
	if(IsKinematic())
		return static_cast<KinematicMotionState&>(*m_motionState).GetWorldTransform().GetOrigin();
	return pragma::physics::BtCollisionObject::GetPos();
}
void pragma::physics::BtRigidBody::SetPos(const Vector3 &pos)
{
	pragma::physics::BtCollisionObject::SetPos(pos);
	if(IsKinematic())
	{
		auto &t = static_cast<KinematicMotionState&>(*m_motionState).GetWorldTransform();
		t.SetOrigin(pos);
	}
}
Quat pragma::physics::BtRigidBody::GetRotation() const
{
	if(IsKinematic())
		return static_cast<KinematicMotionState&>(*m_motionState).GetWorldTransform().GetRotation();
	return pragma::physics::BtCollisionObject::GetRotation();
}
void pragma::physics::BtRigidBody::SetRotation(const Quat &rot)
{
	pragma::physics::BtCollisionObject::SetRotation(rot);
	if(IsKinematic())
	{
		auto &t = static_cast<KinematicMotionState&>(*m_motionState).GetWorldTransform();
		t.SetRotation(rot);
	}
}
void pragma::physics::BtRigidBody::PreSimulate()
{
	auto &nw = m_physEnv.GetNetworkState();
	auto *game = nw.GetGameState();
	auto dt = game->DeltaTickTime();
	if(dt > 0.0)
		SetLinearVelocity(GetLinearVelocity() +GetLinearCorrectionVelocity() /static_cast<float>(dt));
	if(IsKinematic())
	{
		auto &transform = static_cast<KinematicMotionState*>(m_motionState.get())->GetWorldTransform();
		transform.SetOrigin(transform.GetOrigin() +GetLinearVelocity() *static_cast<float>(dt));
		auto angVel = GetAngularVelocity() *static_cast<float>(dt);
		EulerAngles ang {
			static_cast<float>(umath::rad_to_deg(angVel.x)),
			static_cast<float>(umath::rad_to_deg(angVel.y)),
			static_cast<float>(umath::rad_to_deg(angVel.z))
		};
		transform.SetRotation(transform.GetRotation() *uquat::create(ang));
	}
	pragma::physics::ICollisionObject::PreSimulate();
}
void pragma::physics::BtRigidBody::PostSimulate()
{
	auto &nw = GetBtEnv().GetNetworkState();
	auto *game = nw.GetGameState();
	auto dt = game->DeltaTickTime();
	if(dt > 0.0)
		SetLinearVelocity(GetLinearVelocity() -GetLinearCorrectionVelocity() /static_cast<float>(dt));
	ResetLinearCorrectionVelocity();
	pragma::physics::ICollisionObject::PostSimulate();
}

/////////////////////////

pragma::physics::BtSoftBody::BtSoftBody(IEnvironment &env,std::unique_ptr<btSoftBody> o,IShape &shape,const std::vector<uint16_t> &meshVertIndicesToPhysIndices)
	: ICollisionObject{env,shape},BtCollisionObject{env,std::move(o),shape},ISoftBody{env,shape,meshVertIndicesToPhysIndices},m_rotation{},m_localVertexIndicesToNodeIndices(meshVertIndicesToPhysIndices)
{
	m_nodeIndicesToLocalVertexIndices.resize(meshVertIndicesToPhysIndices.size());
	for(auto i=decltype(meshVertIndicesToPhysIndices.size()){0};i<meshVertIndicesToPhysIndices.size();++i)
		m_nodeIndicesToLocalVertexIndices.at(meshVertIndicesToPhysIndices.at(i)) = i;
	UpdateTotalMass();
	UpdateLinearVelocity();
}

btSoftBody &pragma::physics::BtSoftBody::GetInternalObject() const {return static_cast<btSoftBody&>(BtCollisionObject::GetInternalObject());}

void pragma::physics::BtSoftBody::AppendAnchor(uint32_t nodeId,pragma::physics::IRigidBody &body,const Vector3 &localPivot,bool bDisableCollision,float influence)
{
	assert(nodeId < GetInternalObject().m_nodes.size());
	if(nodeId >= GetInternalObject().m_nodes.size())
	{
		Con::cwar<<"WARNING: Attempted to add soft-body anchor for invalid node "<<nodeId<<"! Skipping..."<<Con::endl;
		return;
	}
	GetInternalObject().appendAnchor(nodeId,&dynamic_cast<BtRigidBody&>(body).GetInternalObject(),uvec::create_bt(localPivot) *BtEnvironment::WORLD_SCALE,bDisableCollision,influence);
}
void pragma::physics::BtSoftBody::AppendAnchor(uint32_t nodeId,pragma::physics::IRigidBody &body,bool bDisableCollision,float influence)
{
	assert(nodeId < GetInternalObject().m_nodes.size());
	if(nodeId >= GetInternalObject().m_nodes.size())
	{
		Con::cwar<<"WARNING: Attempted to add soft-body anchor for invalid node "<<nodeId<<"! Skipping..."<<Con::endl;
		return;
	}
	GetInternalObject().appendAnchor(nodeId,&dynamic_cast<BtRigidBody&>(body).GetInternalObject(),bDisableCollision,influence);
}
void pragma::physics::BtSoftBody::DoAddWorldObject()
{
#if PHYS_USE_SOFT_RIGID_DYNAMICS_WORLD == 1
	auto *world = GetBtEnv().GetWorld();
	world->addSoftBody(&GetInternalObject(),umath::to_integral(m_collisionFilterGroup),umath::to_integral(m_collisionFilterMask));
#endif
}
pragma::physics::BtSoftBody *pragma::physics::BtSoftBody::GetBtSoftBody() {return this;}
const btAlignedObjectArray<btSoftBody::Node> &pragma::physics::BtSoftBody::GetNodes() const {return GetInternalObject().m_nodes;}
uint32_t pragma::physics::BtSoftBody::GetNodeCount() const {return GetInternalObject().m_nodes.size();}

void pragma::physics::BtSoftBody::GetAABB(Vector3 &min,Vector3 &max) const
{
	btVector3 btMin,btMax;
	GetInternalObject().getAabb(btMin,btMax);
	min.x = static_cast<float>(btMin.x());
	min.y = static_cast<float>(btMin.y());
	min.z = static_cast<float>(btMin.z());

	max.x = static_cast<float>(btMax.x());
	max.y = static_cast<float>(btMax.y());
	max.z = static_cast<float>(btMax.z());
	min /= BtEnvironment::WORLD_SCALE;
	max /= BtEnvironment::WORLD_SCALE;
}
void pragma::physics::BtSoftBody::AddAeroForceToNode(int32_t node,const Vector3 &force) {GetInternalObject().addAeroForceToNode(uvec::create_bt(force) *BtEnvironment::WORLD_SCALE,node);}
void pragma::physics::BtSoftBody::AddAeroForceToFace(int32_t face,const Vector3 &force) {GetInternalObject().addAeroForceToFace(uvec::create_bt(force) *BtEnvironment::WORLD_SCALE,face);}
void pragma::physics::BtSoftBody::AddForce(const Vector3 &force) {GetInternalObject().addForce(uvec::create_bt(force) *BtEnvironment::WORLD_SCALE);}
void pragma::physics::BtSoftBody::AddForce(uint32_t node,const Vector3 &force) {GetInternalObject().addForce(uvec::create_bt(force) *BtEnvironment::WORLD_SCALE,node);}
void pragma::physics::BtSoftBody::AddLinearVelocity(const Vector3 &vel) {GetInternalObject().addVelocity(uvec::create_bt(vel) *BtEnvironment::WORLD_SCALE);}
void pragma::physics::BtSoftBody::AddLinearVelocity(uint32_t node,const Vector3 &vel) {GetInternalObject().addVelocity(uvec::create_bt(vel) *BtEnvironment::WORLD_SCALE,node);}
float pragma::physics::BtSoftBody::GetFriction() const {return static_cast<float>(GetInternalObject().getFriction());}
float pragma::physics::BtSoftBody::GetHitFraction() const {return static_cast<float>(GetInternalObject().getHitFraction());}
float pragma::physics::BtSoftBody::GetRollingFriction() const {return static_cast<float>(GetInternalObject().getRollingFriction());}
Vector3 pragma::physics::BtSoftBody::GetAnisotropicFriction() const {return uvec::create(GetInternalObject().getAnisotropicFriction());}
void pragma::physics::BtSoftBody::SetFriction(float friction) {GetInternalObject().setFriction(friction);}
void pragma::physics::BtSoftBody::SetHitFraction(float fraction) {GetInternalObject().setHitFraction(fraction);}
void pragma::physics::BtSoftBody::SetRollingFriction(float friction) {GetInternalObject().setRollingFriction(friction);}
void pragma::physics::BtSoftBody::SetAnisotropicFriction(const Vector3 &friction) {GetInternalObject().setAnisotropicFriction(uvec::create_bt(friction));}
float pragma::physics::BtSoftBody::GetMass(int32_t node) const {return static_cast<float>(GetInternalObject().getMass(node));}
float pragma::physics::BtSoftBody::GetMass() const {return m_totalMass;}
float pragma::physics::BtSoftBody::GetRestitution() const {return static_cast<float>(GetInternalObject().getRestitution());}
float pragma::physics::BtSoftBody::GetRestLengthScale() const {return static_cast<float>(GetInternalObject().getRestLengthScale());}
Vector3 pragma::physics::BtSoftBody::GetWindVelocity() const {return uvec::create(GetInternalObject().getWindVelocity() /BtEnvironment::WORLD_SCALE);}
void pragma::physics::BtSoftBody::SetMass(int32_t node,float mass) {GetInternalObject().setMass(node,mass); UpdateTotalMass();}
void pragma::physics::BtSoftBody::SetMass(float mass) {GetInternalObject().setTotalMass(mass); UpdateTotalMass();}
void pragma::physics::BtSoftBody::SetRestitution(float rest) {GetInternalObject().setRestitution(rest);}
void pragma::physics::BtSoftBody::SetRestLengthScale(float scale) {GetInternalObject().setRestLengthScale(scale);}
void pragma::physics::BtSoftBody::SetWindVelocity(const Vector3 &vel) {GetInternalObject().setWindVelocity(uvec::create_bt(vel) *BtEnvironment::WORLD_SCALE);}
void pragma::physics::BtSoftBody::SetLinearVelocity(const Vector3 &vel) {GetInternalObject().setVelocity(uvec::create_bt(vel) *BtEnvironment::WORLD_SCALE);}
void pragma::physics::BtSoftBody::SetVolumeDensity(float density) {GetInternalObject().setVolumeDensity(density);}
void pragma::physics::BtSoftBody::SetVolumeMass(float mass) {GetInternalObject().setVolumeMass(mass);}
float pragma::physics::BtSoftBody::GetVolume() const {return static_cast<float>(GetInternalObject().getVolume());}
void pragma::physics::BtSoftBody::SetDensity(float density) {GetInternalObject().setTotalDensity(density);}
const Vector3 &pragma::physics::BtSoftBody::GetLinearVelocity() const {return m_linearVelocity;}
void pragma::physics::BtSoftBody::UpdateTotalMass() {m_totalMass = GetInternalObject().getTotalMass();}
void pragma::physics::BtSoftBody::UpdateLinearVelocity()
{
	auto linVel = btVector3{0.f,0.f,0.f};
	for(auto i=decltype(GetInternalObject().m_nodes.size()){0};i<GetInternalObject().m_nodes.size();++i)
	{
		auto &node = GetInternalObject().m_nodes.at(i);
		linVel += node.m_v;
	}
	linVel /= static_cast<float>(GetInternalObject().m_nodes.size());
	m_linearVelocity = uvec::create(linVel /BtEnvironment::WORLD_SCALE);
}

void pragma::physics::BtSoftBody::SetAnchorsHardness(float val) {GetInternalObject().m_cfg.kAHR = val;}
void pragma::physics::BtSoftBody::SetRigidContactsHardness(float val) {GetInternalObject().m_cfg.kCHR = val;}
void pragma::physics::BtSoftBody::SetDynamicFrictionCoefficient(float val) {GetInternalObject().m_cfg.kDF = val;}
void pragma::physics::BtSoftBody::SetDragCoefficient(float val) {GetInternalObject().m_cfg.kDG = val;}
void pragma::physics::BtSoftBody::SetDampingCoefficient(float val) {GetInternalObject().m_cfg.kDP = val;}
void pragma::physics::BtSoftBody::SetKineticContactsHardness(float val) {GetInternalObject().m_cfg.kKHR = val;}
void pragma::physics::BtSoftBody::SetLiftCoefficient(float val) {GetInternalObject().m_cfg.kLF = val;}
void pragma::physics::BtSoftBody::SetPoseMatchingCoefficient(float val) {GetInternalObject().m_cfg.kMT = val;}
void pragma::physics::BtSoftBody::SetPressureCoefficient(float val) {GetInternalObject().m_cfg.kPR = val;}
void pragma::physics::BtSoftBody::SetSoftContactsHardness(float val) {GetInternalObject().m_cfg.kSHR = val;}
void pragma::physics::BtSoftBody::SetSoftVsKineticHardness(float val) {GetInternalObject().m_cfg.kSKHR_CL = val;}
void pragma::physics::BtSoftBody::SetSoftVsRigidImpulseSplitK(float val) {GetInternalObject().m_cfg.kSK_SPLT_CL = val;}
void pragma::physics::BtSoftBody::SetSoftVsRigidHardness(float val) {GetInternalObject().m_cfg.kSRHR_CL = val;}
void pragma::physics::BtSoftBody::SetSoftVsRigidImpulseSplitR(float val) {GetInternalObject().m_cfg.kSR_SPLT_CL = val;}
void pragma::physics::BtSoftBody::SetSoftVsSoftHardness(float val) {GetInternalObject().m_cfg.kSSHR_CL = val;}
void pragma::physics::BtSoftBody::SetSoftVsRigidImpulseSplitS(float val) {GetInternalObject().m_cfg.kSS_SPLT_CL = val;}
void pragma::physics::BtSoftBody::SetVolumeConversationCoefficient(float val) {GetInternalObject().m_cfg.kVC = val;}
void pragma::physics::BtSoftBody::SetVelocitiesCorrectionFactor(float val) {GetInternalObject().m_cfg.kVCF = val;}

float pragma::physics::BtSoftBody::GetAnchorsHardness() const {return GetInternalObject().m_cfg.kAHR;}
float pragma::physics::BtSoftBody::GetRigidContactsHardness() const {return GetInternalObject().m_cfg.kCHR;}
float pragma::physics::BtSoftBody::GetDynamicFrictionCoefficient() const {return GetInternalObject().m_cfg.kDF;}
float pragma::physics::BtSoftBody::GetDragCoefficient() const {return GetInternalObject().m_cfg.kDG;}
float pragma::physics::BtSoftBody::GetDampingCoefficient() const {return GetInternalObject().m_cfg.kDP;}
float pragma::physics::BtSoftBody::GetKineticContactsHardness() const {return GetInternalObject().m_cfg.kKHR;}
float pragma::physics::BtSoftBody::GetLiftCoefficient() const {return GetInternalObject().m_cfg.kLF;}
float pragma::physics::BtSoftBody::GetPoseMatchingCoefficient() const {return GetInternalObject().m_cfg.kMT;}
float pragma::physics::BtSoftBody::GetPressureCoefficient() const {return GetInternalObject().m_cfg.kPR;}
float pragma::physics::BtSoftBody::GetSoftContactsHardness() const {return GetInternalObject().m_cfg.kSHR;}
float pragma::physics::BtSoftBody::GetSoftVsKineticHardness() const {return GetInternalObject().m_cfg.kSKHR_CL;}
float pragma::physics::BtSoftBody::GetSoftVsRigidImpulseSplitK() const {return GetInternalObject().m_cfg.kSK_SPLT_CL;}
float pragma::physics::BtSoftBody::GetSoftVsRigidHardness() const {return GetInternalObject().m_cfg.kSRHR_CL;}
float pragma::physics::BtSoftBody::GetSoftVsRigidImpulseSplitR() const {return GetInternalObject().m_cfg.kSR_SPLT_CL;}
float pragma::physics::BtSoftBody::GetSoftVsSoftHardness() const {return GetInternalObject().m_cfg.kSSHR_CL;}
float pragma::physics::BtSoftBody::GetSoftVsRigidImpulseSplitS() const {return GetInternalObject().m_cfg.kSS_SPLT_CL;}
float pragma::physics::BtSoftBody::GetVolumeConversationCoefficient() const {return GetInternalObject().m_cfg.kVC;}
float pragma::physics::BtSoftBody::GetVelocitiesCorrectionFactor() const {return GetInternalObject().m_cfg.kVCF;}

void pragma::physics::BtSoftBody::SetMaterialAngularStiffnessCoefficient(uint32_t matId,float val)
{
	if(matId >= GetInternalObject().m_materials.size())
		return;
	GetInternalObject().m_materials.at(matId)->m_kAST = val;
}
void pragma::physics::BtSoftBody::SetMaterialLinearStiffnessCoefficient(uint32_t matId,float val)
{
	if(matId >= GetInternalObject().m_materials.size())
		return;
	GetInternalObject().m_materials.at(matId)->m_kLST = val;
}
void pragma::physics::BtSoftBody::SetMaterialVolumeStiffnessCoefficient(uint32_t matId,float val)
{
	if(matId >= GetInternalObject().m_materials.size())
		return;
	GetInternalObject().m_materials.at(matId)->m_kVST = val;
}
float pragma::physics::BtSoftBody::GetMaterialAngularStiffnessCoefficient(uint32_t matId) const
{
	if(matId >= GetInternalObject().m_materials.size())
		return 0.f;
	return GetInternalObject().m_materials.at(matId)->m_kAST;
}
float pragma::physics::BtSoftBody::GetMaterialLinearStiffnessCoefficient(uint32_t matId) const
{
	if(matId >= GetInternalObject().m_materials.size())
		return 0.f;
	return GetInternalObject().m_materials.at(matId)->m_kLST;
}
float pragma::physics::BtSoftBody::GetMaterialVolumeStiffnessCoefficient(uint32_t matId) const
{
	if(matId >= GetInternalObject().m_materials.size())
		return 0.f;
	return GetInternalObject().m_materials.at(matId)->m_kVST;
}

void pragma::physics::BtSoftBody::RemoveWorldObject()
{
#if PHYS_USE_SOFT_RIGID_DYNAMICS_WORLD == 1
	auto *world = GetBtEnv().GetWorld();
	world->removeSoftBody(&GetInternalObject());
#endif
}

Vector3 pragma::physics::BtSoftBody::GetPos() const
{
	Vector3 min,max;
	GetAABB(min,max);
	return (min +max) *0.5f;
}

void pragma::physics::BtSoftBody::SetPos(const Vector3 &pos)
{
	auto posCur = GetPos();
	auto t = GetInternalObject().getWorldTransform();
	t.setOrigin(uvec::create_bt(posCur) *BtEnvironment::WORLD_SCALE);
	auto inv = t.inverse();
	t.setOrigin(uvec::create_bt(pos) *BtEnvironment::WORLD_SCALE);
	GetInternalObject().transform(inv *t);
}
void pragma::physics::BtSoftBody::SetRotation(const Quat &rot)
{
	auto transformRot = uquat::get_inverse(m_rotation) *rot;
	btTransform t = {};
	t.setIdentity();
	t.setRotation(uquat::create_bt(transformRot));
	GetInternalObject().transform(t);

	m_rotation = rot;
}

Quat pragma::physics::BtSoftBody::GetRotation() const {return m_rotation;}

void pragma::physics::BtSoftBody::SetWorldTransform(const Transform &t)
{
	auto &bt = BtEnvironment::CreateBtTransform(t);
	GetInternalObject().transform(GetInternalObject().getWorldTransform().inverse() *bt);
}

const std::vector<uint16_t> &pragma::physics::BtSoftBody::GetMeshVertexIndicesToLocalIndices() const {return m_meshVertexIndicesToLocalVertexIndices;}
const std::vector<uint16_t> &pragma::physics::BtSoftBody::GetLocalVertexIndicesToNodeIndices() const {return m_localVertexIndicesToNodeIndices;}
const std::vector<uint16_t> &pragma::physics::BtSoftBody::GetLocalVertexIndicesToMeshVertexIndices() const {return m_localVertexIndicesToMeshVertexIndices;}
const std::vector<uint16_t> &pragma::physics::BtSoftBody::GetNodeIndicesToLocalVertexIndices() const {return m_nodeIndicesToLocalVertexIndices;}
void pragma::physics::BtSoftBody::AddVelocity(const Vector3 &vel)
{
	GetInternalObject().addVelocity(uvec::create_bt(vel) *BtEnvironment::WORLD_SCALE);
}
bool pragma::physics::BtSoftBody::MeshVertexIndexToLocalVertexIndex(uint16_t meshVertexIndex,uint16_t &localIndex) const
{
	if(meshVertexIndex >= m_meshVertexIndicesToLocalVertexIndices.size())
		return false;
	localIndex = m_meshVertexIndicesToLocalVertexIndices.at(meshVertexIndex);
	return true;
}
bool pragma::physics::BtSoftBody::LocalVertexIndexToMeshVertexIndex(uint16_t localIndex,uint16_t &meshVertexIndex) const
{
	if(localIndex >= m_localVertexIndicesToMeshVertexIndices.size())
		return false;
	meshVertexIndex = m_localVertexIndicesToMeshVertexIndices.at(localIndex);
	return true;
}
bool pragma::physics::BtSoftBody::LocalVertexIndexToNodeIndex(uint16_t localVertexIndex,uint16_t &nodeIndex) const
{
	if(localVertexIndex >= m_localVertexIndicesToNodeIndices.size())
		return false;
	nodeIndex = m_localVertexIndicesToNodeIndices.at(localVertexIndex);
	return true;
}
bool pragma::physics::BtSoftBody::NodeIndexToLocalVertexIndex(uint16_t nodeIndex,uint16_t &localVertexIndex) const
{
	if(nodeIndex >= m_nodeIndicesToLocalVertexIndices.size())
		return false;
	localVertexIndex = m_nodeIndicesToLocalVertexIndices.at(nodeIndex);
	return true;
}

bool pragma::physics::BtSoftBody::MeshVertexIndexToNodeIndex(uint16_t meshVertexIndex,uint16_t &nodeIndex) const
{
	uint16_t localIndex = 0u;
	if(MeshVertexIndexToLocalVertexIndex(meshVertexIndex,localIndex) == false)
		return false;
	return LocalVertexIndexToNodeIndex(localIndex,nodeIndex);
}
bool pragma::physics::BtSoftBody::NodeIndexToMeshVertexIndex(uint16_t nodeIndex,uint16_t &meshVertexIndex) const
{
	uint16_t localIndex = 0u;
	if(NodeIndexToLocalVertexIndex(nodeIndex,localIndex) == false)
		return false;
	return LocalVertexIndexToMeshVertexIndex(localIndex,meshVertexIndex);
}
void pragma::physics::BtSoftBody::SetSubMesh(const ModelSubMesh &subMesh,const std::vector<uint16_t> &meshVertexIndicesToLocalVertexIndices)
{
	m_subMesh = const_cast<ModelSubMesh&>(subMesh).shared_from_this();
	m_meshVertexIndicesToLocalVertexIndices = meshVertexIndicesToLocalVertexIndices;

	m_localVertexIndicesToMeshVertexIndices.clear();
	m_localVertexIndicesToMeshVertexIndices.resize(GetNodeCount(),std::numeric_limits<uint16_t>::max());
	for(auto i=decltype(m_meshVertexIndicesToLocalVertexIndices.size()){0};i<m_meshVertexIndicesToLocalVertexIndices.size();++i)
	{
		auto localIdx = meshVertexIndicesToLocalVertexIndices.at(i);
		if(localIdx == std::numeric_limits<uint16_t>::max())
			continue;
		if(localIdx >= m_localVertexIndicesToMeshVertexIndices.size())
		{
			Con::cwar<<"WARNING: Invalid soft-body node index "<<localIdx<<"! Skipping..."<<Con::endl;
			continue;
		}
		m_localVertexIndicesToMeshVertexIndices.at(localIdx) = i;
	}
}

/////////////

pragma::physics::BtGhostObject::BtGhostObject(IEnvironment &env,std::unique_ptr<btPairCachingGhostObject> o,IShape &shape)
	: ICollisionObject{env,shape},BtCollisionObject(env,std::move(o),shape),IGhostObject{env,shape}
{}

btPairCachingGhostObject &pragma::physics::BtGhostObject::GetInternalObject() const {return static_cast<btPairCachingGhostObject&>(BtCollisionObject::GetInternalObject());}

pragma::physics::BtGhostObject *pragma::physics::BtGhostObject::GetBtGhostObject() {return this;}

void pragma::physics::BtGhostObject::DoAddWorldObject()
{
	if(m_bSpawned == false)
		return;
	RemoveWorldObject();
	auto *world = GetBtEnv().GetWorld();
	world->addCollisionObject(m_collisionObject.get(),umath::to_integral(m_collisionFilterGroup),umath::to_integral(m_collisionFilterMask));//,btBroadphaseProxy::CharacterFilter,btBroadphaseProxy::StaticFilter | btBroadphaseProxy::DefaultFilter);
}

void pragma::physics::BtGhostObject::RemoveWorldObject()
{
	auto *world = GetBtEnv().GetWorld();
	world->removeCollisionObject(m_collisionObject.get());
}
#pragma optimize("",on)
