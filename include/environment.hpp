/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef __PR_BT_ENVIRONMENT_HPP__
#define __PR_BT_ENVIRONMENT_HPP__

#include "common.hpp"
#include <pragma/physics/environment.hpp>
#include <unordered_set>
#include <queue>
#include <sharedutils/util_hash.hpp>
#ifdef __linux__
#include <BulletSoftBody/btSoftBody.h>
#include <BulletSoftBody/btSoftBodySolvers.h>
#endif

#define PHYS_USE_SOFT_RIGID_DYNAMICS_WORLD 1

#if PHYS_USE_SOFT_RIGID_DYNAMICS_WORLD == 1
using btWorldType = btSoftRigidDynamicsWorld;
#else
using btWorldType = btDiscreteDynamicsWorld;
#endif

class PhysOverlapFilterCallback;
namespace pragma::physics
{
	class BtRigidBody;
	class IBase;
	class BtDebugDrawer;

	using CollisionContact = std::pair<const btCollisionObject*,const btCollisionObject*>;
	struct CollisionContactHash
	{
		std::size_t operator() (const CollisionContact &pair) const {
			return util::hash_combine<const btCollisionObject*>(util::hash_combine<const btCollisionObject*>(0,pair.first),pair.second);
		}
	};
	using CollisionContactList = std::unordered_set<CollisionContact,CollisionContactHash>;
	struct ContactMap
	{
		void AddContact(const btCollisionObject &a,const btCollisionObject &b);
		void RemoveContact(const btCollisionObject &a,const btCollisionObject &b);
		void RemoveContacts(const btCollisionObject &o);
		void UpdateContacts(const CollisionContactList &newContacts);
		~ContactMap();
	private:
		CollisionContactList::iterator ClearContact(CollisionContactList::iterator it);
		CollisionContactList m_contacts;
	};

	class BtEnvironment
		: public pragma::physics::IEnvironment
	{
	public:
		static const double WORLD_SCALE;
		static const double WORLD_SCALE_SQR; // = WORLD_SCALE^2, required for torque
		static const float CCD_MOTION_THRESHOLD;
		static const float CCD_SWEPT_SPHERE_RADIUS;
	public:
		static void SimulationCallback(btDynamicsWorld *world,btScalar timeStep);
		static umath::Transform CreateTransform(const btTransform &btTransform);
		static btTransform CreateBtTransform(const umath::Transform &btTransform);
		static Vector3 ToPragmaPosition(const btVector3 &pos);
		static btVector3 ToBtPosition(const Vector3 &pos);
		static Vector3 ToPragmaNormal(const btVector3 &n);
		static btVector3 ToBtNormal(const Vector3 &n);
		static double ToPragmaDistance(btScalar d);
		static btScalar ToBtDistance(double d);
		static Color ToPragmaColor(const btVector3 &col);

		BtEnvironment(NetworkState &state);
		virtual ~BtEnvironment() override;

		BtRigidBody &ToBtType(IRigidBody &body);

		virtual float GetWorldScale() const override;

		virtual void StartProfiling() override;
		virtual void EndProfiling() override;

		virtual util::TSharedHandle<IFixedConstraint> CreateFixedConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB) override;
		virtual util::TSharedHandle<IBallSocketConstraint> CreateBallSocketConstraint(IRigidBody &a,const Vector3 &pivotA,IRigidBody &b,const Vector3 &pivotB) override;
		virtual util::TSharedHandle<IHingeConstraint> CreateHingeConstraint(IRigidBody &a,const Vector3 &pivotA,IRigidBody &b,const Vector3 &pivotB,const Vector3 &axis) override;
		virtual util::TSharedHandle<ISliderConstraint> CreateSliderConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB) override;
		virtual util::TSharedHandle<IConeTwistConstraint> CreateConeTwistConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB) override;
		virtual util::TSharedHandle<IDoFConstraint> CreateDoFConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB) override;
		virtual util::TSharedHandle<IDoFSpringConstraint> CreateDoFSpringConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB) override;

		virtual util::TSharedHandle<IController> CreateCapsuleController(float halfWidth,float halfHeight,float stepHeight,float slopeLimitDeg=DEFAULT_CHARACTER_SLOPE_LIMIT,const umath::Transform &startTransform={}) override;
		virtual util::TSharedHandle<IController> CreateBoxController(const Vector3 &halfExtents,float stepHeight,float slopeLimitDeg=DEFAULT_CHARACTER_SLOPE_LIMIT,const umath::Transform &startTransform={}) override;
		virtual util::TSharedHandle<ICollisionObject> CreateCollisionObject(IShape &shape) override;
		virtual util::TSharedHandle<IRigidBody> CreateRigidBody(IShape &shape,bool dynamic=true) override;
		virtual util::TSharedHandle<ISoftBody> CreateSoftBody(const PhysSoftBodyInfo &info,float mass,const std::vector<Vector3> &verts,const std::vector<uint16_t> &indices,std::vector<uint16_t> &indexTranslations) override;
		virtual util::TSharedHandle<IGhostObject> CreateGhostObject(IShape &shape) override;

		virtual std::shared_ptr<IConvexShape> CreateCapsuleShape(float halfWidth,float halfHeight,const IMaterial &mat) override;
		virtual std::shared_ptr<IConvexShape> CreateBoxShape(const Vector3 &halfExtents,const IMaterial &mat) override;
		virtual std::shared_ptr<IConvexShape> CreateCylinderShape(float radius,float height,const IMaterial &mat) override;
		std::shared_ptr<ICompoundShape> CreateTorusShape(uint32_t subdivisions,double outerRadius,double innerRadius,const IMaterial &mat);
		virtual std::shared_ptr<IConvexShape> CreateSphereShape(float radius,const IMaterial &mat) override;
		virtual std::shared_ptr<IConvexHullShape> CreateConvexHullShape(const IMaterial &mat) override;
		virtual std::shared_ptr<ITriangleShape> CreateTriangleShape(const IMaterial &mat) override;
		virtual std::shared_ptr<ICompoundShape> CreateCompoundShape(std::vector<IShape*> &shapes) override;
		virtual std::shared_ptr<IShape> CreateHeightfieldTerrainShape(uint32_t width,uint32_t length,Scalar maxHeight,uint32_t upAxis,const IMaterial &mat) override;
		virtual std::shared_ptr<IMaterial> CreateMaterial(float staticFriction,float dynamicFriction,float restitution) override;
		virtual util::TSharedHandle<ICollisionObject> CreatePlane(const Vector3 &n,float d,const IMaterial &mat) override;
		virtual util::TSharedHandle<IVehicle> CreateVehicle(const VehicleCreateInfo &vhcDesc) override;

		virtual RemainingDeltaTime DoStepSimulation(float timeStep,int maxSubSteps=1,float fixedTimeStep=(1.f /60.f)) override;

		virtual Bool Overlap(const TraceData &data,std::vector<TraceResult> *results=nullptr) const override;
		virtual Bool RayCast(const TraceData &data,std::vector<TraceResult> *results=nullptr) const override;
		virtual Bool Sweep(const TraceData &data,std::vector<TraceResult> *results=nullptr) const override;

		virtual void RemoveConstraint(IConstraint &constraint) override;
		virtual void RemoveCollisionObject(ICollisionObject &obj) override;
		virtual void RemoveController(IController &controller) override;

		const btWorldType *GetWorld() const;
		btWorldType *GetWorld();
		btDefaultCollisionConfiguration *GetBtCollisionConfiguration();
		btCollisionDispatcher *GetBtCollisionDispatcher();
		btBroadphaseInterface *GetBtOverlappingPairCache();
		btSequentialImpulseConstraintSolver *GetBtConstraintSolver();
		btSoftBodyWorldInfo *GetBtSoftBodyWorldInfo();
		btSoftBodySolver &GetSoftBodySolver();
		const btSoftBodySolver &GetSoftBodySolver() const;
		
		ContactMap &GetCollisionContactMap() {return m_contactMap;}
		const ContactMap &GetCollisionContactMap() const {return const_cast<BtEnvironment*>(this)->GetCollisionContactMap();}

		// For internal or debugging purposes only!
		util::TSharedHandle<IFixedConstraint> AddFixedConstraint(std::unique_ptr<btFixedConstraint> c);
		util::TSharedHandle<IBallSocketConstraint> AddBallSocketConstraint(std::unique_ptr<btPoint2PointConstraint> c);
		util::TSharedHandle<IHingeConstraint> AddHingeConstraint(std::unique_ptr<btHingeConstraint> c);
		util::TSharedHandle<ISliderConstraint> AddSliderConstraint(std::unique_ptr<btSliderConstraint> c);
		util::TSharedHandle<IConeTwistConstraint> AddConeTwistConstraint(std::unique_ptr<btConeTwistConstraint> c);
		util::TSharedHandle<IDoFConstraint> AddDoFConstraint(std::unique_ptr<btGeneric6DofConstraint> c);
		util::TSharedHandle<IDoFSpringConstraint> AddDoFSpringConstraint(std::unique_ptr<btGeneric6DofSpring2Constraint> c);

		uint64_t GetCurrentSimulationStepIndex() const {return m_curSimStepIndex;}
		void PushEvent(const std::function<void()> &ev) {m_events.push(ev);}
	protected:
		virtual void UpdateSurfaceTypes() override;
		virtual void OnVisualDebuggerChanged(pragma::physics::IVisualDebugger *debugger) override;
		std::unique_ptr<btWorldType> m_btWorld = nullptr;
		std::unique_ptr<btDefaultCollisionConfiguration> m_btCollisionConfiguration = nullptr;
		std::unique_ptr<btCollisionDispatcher> m_btDispatcher = nullptr;
		std::unique_ptr<btBroadphaseInterface> m_btOverlappingPairCache = nullptr;
		std::unique_ptr<PhysOverlapFilterCallback> m_overlapFilterCallback;
		std::unique_ptr<btSequentialImpulseConstraintSolver> m_btSolver = nullptr;
		std::unique_ptr<btGhostPairCallback> m_btGhostPairCallback = nullptr;
		std::unique_ptr<btSoftBodySolver> m_softBodySolver = nullptr;
		std::unique_ptr<btSoftBodyWorldInfo> m_softBodyWorldInfo;
		std::unique_ptr<BtDebugDrawer> m_btDebugDrawer = nullptr;
		ContactMap m_contactMap {};
		uint64_t m_curSimStepIndex = 0;
		std::queue<std::function<void()>> m_events;

		void AddAction(btActionInterface *action);

		void SimulationCallback(double timeStep);
	};
};

#if 0
// Vehicle
btRaycastVehicle::btVehicleTuning m_tuning = {};
std::unique_ptr<btRaycastVehicle> m_vhcRaycast = nullptr;

PhysVehicleRaycaster::PhysVehicleRaycaster(Game *game,pragma::BaseVehicleComponent &vhc)
	: m_game(game),m_vehicle(vhc),m_entity(vhc.GetEntity())
{
#if USE_CONVEX_WHEELS == 1
	auto radius = vhc->GetWheelRadius();
	//m_shape = m_game->GetPhysicsEnvironment()->CreateCylinderShape(radius,radius *0.5f);
	m_shape = m_game->GetPhysicsEnvironment()->CreateSphereShape(radius);
#endif
}
PhysVehicleRaycaster::~PhysVehicleRaycaster()
{}
void *PhysVehicleRaycaster::castRay(const btVector3 &from,const btVector3 &to, btVehicleRaycasterResult &result)
{
#ifdef ENABLE_DEPRECATED_PHYSICS
#if USE_CONVEX_WHEELS == 1
	const auto axisRot = static_cast<float>(umath::sin(umath::deg_to_rad(45.0)));
	const auto shapeRot = uquat::create(EulerAngles(0.f,0.f,90.f));//Quat{axisRot,0.f,0.f,axisRot}; // 90 degree rotation
#endif
	auto start = Vector3(from.x(),from.y(),from.z()) /static_cast<float>(PhysEnv::WORLD_SCALE);
	auto end = Vector3(to.x(),to.y(),to.z()) /static_cast<float>(PhysEnv::WORLD_SCALE);
	TraceData trData;
	trData.SetSource(start);
	trData.SetTarget(end);
	trData.SetCollisionFilterMask(CollisionMask::All &~CollisionMask::Vehicle);
#if USE_CONVEX_WHEELS == 1
	//trData.SetFilter(dynamic_cast<BaseEntity*>(m_game->GetWorld())->GetHandle());
	trData.SetSourceRotation(m_entity->GetOrientation() *shapeRot);
	trData.SetSource(m_shape->GetConvexShape());
	auto r = m_game->Sweep(trData);
#else
	auto r = m_game->RayCast(trData);
#endif
	if(r.hit == true && r.collisionObj.IsValid())
	{
		auto *body = btRigidBody::upcast(r.collisionObj->GetCollisionObject());
		if(body != nullptr && body->hasContactResponse())
		{
#if USE_CONVEX_WHEELS == 1
			auto hitPos = r.position +r.normal *19.5f; // TODO
			auto d = uvec::distance(start,end);
			auto fraction = (r.fraction *d) /(d +19.5f);
#else
			auto &hitPos = r.position;
			auto fraction = r.fraction;
#endif
			result.m_hitPointInWorld = btVector3(hitPos.x,hitPos.y,hitPos.z) *PhysEnv::WORLD_SCALE;
			result.m_hitNormalInWorld = btVector3(r.normal.x,r.normal.y,r.normal.z);
			result.m_hitNormalInWorld.normalize();
			result.m_distFraction = fraction;
			return (void*)body;
		}
	}
	return 0;
#else
	return nullptr;
#endif
}

BaseVehicleComponent::WheelData::WheelData()
	: hWheel()
{}

void BaseVehicleComponent::OnPhysicsInitialized()
{
	auto &ent = GetEntity();
	auto *nw = ent.GetNetworkState();
	auto *game = nw->GetGameState();
	auto *physEnv = game->GetPhysicsEnvironment();
	if(physEnv == nullptr)
		return;
	//auto vhc = physEnv->CreateVehicle();
	//if(vhc == nullptr)
	//		return;

#ifdef ENABLE_DEPRECATED_PHYSICS
	auto &ent = GetEntity();
	auto pPhysComponent = ent.GetPhysicsComponent();
	auto *phys = pPhysComponent.valid() ? pPhysComponent->GetPhysicsObject() : nullptr;
	if(phys == nullptr || !phys->IsRigid())
		return;
	auto *nw = ent.GetNetworkState();
	auto *game = nw->GetGameState();
	auto *physEnv = game->GetPhysicsEnvironment();
	auto *physWorld = physEnv->GetWorld();
	auto *physRigid = static_cast<RigidPhysObj*>(phys);

	for(auto &o : physRigid->GetCollisionObjects())
		o->SetContactProcessingThreshold(0.f); // Without this vehicles stumble over edges between meshes frequently

	auto *rigidBody = physRigid->GetRigidBody();
	if(rigidBody == nullptr)
		return;
	auto *btRigidBody = rigidBody->GetRigidBody();
	m_vhcRayCaster = std::make_unique<PhysVehicleRaycaster>(game,*this);
	rigidBody->SetActivationState(DISABLE_DEACTIVATION);
	m_vhcRaycast = std::make_unique<btRaycastVehicle>(m_tuning,btRigidBody,m_vhcRayCaster.get());
	m_vhcRaycast->setCoordinateSystem(0,1,2);
	physWorld->addVehicle(m_vhcRaycast.get());

	InitializeWheels();
#endif
}

void BaseVehicleComponent::OnPhysicsDestroyed()
{
#ifdef ENABLE_DEPRECATED_PHYSICS
	if(m_vhcRaycast != nullptr)
	{
		auto &ent = GetEntity();
		auto *nw = ent.GetNetworkState();
		auto *game = nw->GetGameState();
		auto *physEnv = game->GetPhysicsEnvironment();
		auto *physWorld = physEnv->GetWorld();
		physWorld->removeVehicle(m_vhcRaycast.get());
	}
	m_vhcRaycast = nullptr;
	m_vhcRayCaster = nullptr;
#endif
}

BaseVehicleComponent::BaseVehicleComponent(BaseEntity &ent)
	: BaseEntityComponent(ent),m_wheelInfo(),m_engineForce(0.f),
	m_brakeForce(0.f),m_maxEngineForce(100.f),
	m_maxReverseEngineForce(80.f),m_maxBrakeForce(200.f),
	m_acceleration(200.f),m_turnSpeed(90.f),m_maxTurnAngle(45.f),
	m_bFirstPersonCameraEnabled(true),m_bThirdPersonCameraEnabled(true)
{}

BaseVehicleComponent::~BaseVehicleComponent()
{
#ifdef ENABLE_DEPRECATED_PHYSICS
	if(m_cbSteeringWheel.IsValid())
		m_cbSteeringWheel.Remove();
	auto &ent = GetEntity();
	auto *nw = ent.GetNetworkState();
	auto *game = nw->GetGameState();
	auto *physEnv = game->GetPhysicsEnvironment();
	auto *physWorld = physEnv->GetWorld();
	if(m_vhcRaycast != nullptr)
	{
		auto *vhc = m_vhcRaycast.get();
		physWorld->removeVehicle(vhc);
	}
#endif
}

bool BaseVehicleComponent::IsFirstPersonCameraEnabled() const {return m_bFirstPersonCameraEnabled;}
bool BaseVehicleComponent::IsThirdPersonCameraEnabled() const {return m_bThirdPersonCameraEnabled;}
void BaseVehicleComponent::SetFirstPersonCameraEnabled(bool b) {m_bFirstPersonCameraEnabled = b;}
void BaseVehicleComponent::SetThirdPersonCameraEnabled(bool b) {m_bThirdPersonCameraEnabled = b;}

Float BaseVehicleComponent::GetAcceleration() const {return m_acceleration;}
Float BaseVehicleComponent::GetTurnSpeed() const {return m_turnSpeed;}
Float BaseVehicleComponent::GetEngineForce() const {return m_engineForce;}
Float BaseVehicleComponent::GetBrakeForce() const {return m_brakeForce;}
Float BaseVehicleComponent::GetMaxTurnAngle() const {return m_maxTurnAngle;}
Float BaseVehicleComponent::GetSpeedKmh() const
{
#ifdef ENABLE_DEPRECATED_PHYSICS
	if(m_vhcRaycast == nullptr)
		return 0.f;
	return CFloat(m_vhcRaycast->getCurrentSpeedKmHour());
#else
	return 0.f;
#endif
}

std::vector<util::WeakHandle<pragma::BaseWheelComponent>> BaseVehicleComponent::GetWheels() const
{
	std::vector<util::WeakHandle<pragma::BaseWheelComponent>> wheels;
	wheels.reserve(m_wheels.size());
	for(auto &data : m_wheels)
		wheels.push_back(data.hWheel);
	return wheels;
}
util::WeakHandle<pragma::BaseWheelComponent> BaseVehicleComponent::GetWheel(UChar wheelId)
{
	if(wheelId >= m_wheels.size())
		return {};
	return m_wheels[wheelId].hWheel;
}

void BaseVehicleComponent::AttachWheel(UChar wheelId,pragma::BaseWheelComponent *wheel)
{
	auto numWheels = GetWheelCount();
	assert(wheelId < numWheels);
	if(wheelId >= numWheels)
		return;
	if(m_wheels.size() < numWheels)
		m_wheels.resize(numWheels);
	m_wheels[wheelId].hWheel = wheel->GetHandle<pragma::BaseWheelComponent>();
	InitializeWheelEntity(wheel,m_wheels[wheelId]);
}

#ifdef ENABLE_DEPRECATED_PHYSICS
btRaycastVehicle *BaseVehicleComponent::GetBtVehicle() {return (m_vhcRaycast != nullptr) ? m_vhcRaycast.get() : nullptr;}
#endif

void BaseVehicleComponent::DetachWheel(UChar wheelId)
{
	auto numWheels = GetWheelCount();
	if(wheelId >= numWheels)
		return;
	if(wheelId < m_wheels.size())
	{
		auto &data = m_wheels[wheelId];
		if(data.hWheel.valid())
			data.hWheel->Detach();
		m_wheels.erase(m_wheels.begin() +wheelId);
	}
	// TODO Detach bullet wheel
}

void BaseVehicleComponent::InitializeSteeringWheel()
{
	auto *ent = GetSteeringWheel();
	if(ent == nullptr)
		return;
	if(m_cbSteeringWheel.IsValid())
		m_cbSteeringWheel.Remove();
	auto *pAttComponent = static_cast<BaseAttachableComponent*>(ent->FindComponent("attachable").get());
	if(pAttComponent != nullptr)
	{
		m_cbSteeringWheel = pAttComponent->BindEventUnhandled(BaseAttachableComponent::EVENT_ON_ATTACHMENT_UPDATE,[this,pAttComponent](std::reference_wrapper<pragma::ComponentEvent> evData) {
			auto pTrComponentSteeringWheel = pAttComponent->GetEntity().GetTransformComponent();
			if(pTrComponentSteeringWheel.expired())
				return;
			auto ang = EulerAngles(-GetSteeringAngle(),0.f,0.f);
			auto rot = uquat::create(ang);
			auto rotEnt = pTrComponentSteeringWheel->GetOrientation();
			rotEnt = rotEnt *rot;
			pTrComponentSteeringWheel->SetOrientation(rotEnt);
			});
	}
}

void BaseVehicleComponent::SetSteeringWheelModel(const std::string &mdl)
{
	if(!m_steeringWheel.IsValid())
		return;
	auto mdlComponent = m_steeringWheel->GetModelComponent();
	if(mdlComponent.expired())
		return;
	mdlComponent->SetModel(mdl.c_str());
}

void BaseVehicleComponent::ClearDriver()
{
	if(!m_driver.IsValid())
		return;
	m_driver = EntityHandle();
}

void BaseVehicleComponent::SetDriver(BaseEntity *ent)
{
	if(m_driver.IsValid())
		ClearDriver();
	m_driver = ent->GetHandle();
}

BaseEntity *BaseVehicleComponent::GetDriver() {return m_driver.get();}
Bool BaseVehicleComponent::HasDriver() const {return m_driver.IsValid();}

void BaseVehicleComponent::InitializeWheel(const WheelData &data)
{
#ifdef ENABLE_DEPRECATED_PHYSICS
	if(m_vhcRaycast == nullptr)
		return;
	auto &connectionPoint = data.connectionPoint;
	auto &wheelAxle = data.wheelAxle;
	auto bIsFrontWheel = data.bIsFrontWheel;

	auto &wheelDir = GetWheelDirection();
	auto suspensionRestLength = GetMaxSuspensionLength() *PhysEnv::WORLD_SCALE;
	auto wheelRadius = GetWheelRadius();
	auto &info = m_vhcRaycast->addWheel(
		btVector3(connectionPoint.x,connectionPoint.y,connectionPoint.z) *PhysEnv::WORLD_SCALE,
		btVector3(wheelDir.x,wheelDir.y,wheelDir.z),
		btVector3(wheelAxle.x,wheelAxle.y,wheelAxle.z),
		suspensionRestLength,wheelRadius *PhysEnv::WORLD_SCALE,
		m_tuning,bIsFrontWheel
	);
	UNUSED(info);
#endif
}
void BaseVehicleComponent::InitializeWheels()
{
	for(auto &data : m_wheels)
		InitializeWheel(data);
}

Bool BaseVehicleComponent::AddWheel(const Vector3 &connectionPoint,const Vector3 &wheelAxle,Bool bIsFrontWheel,UChar *wheelId,const Vector3 &mdlOffset,const Quat &mdlRotOffset)
{
	m_wheels.push_back({});
	auto &data = m_wheels.back();
	*wheelId = GetWheelCount() -1;
	data.bIsFrontWheel = bIsFrontWheel;
	data.connectionPoint = connectionPoint;
	data.wheelAxle = wheelAxle;
	data.modelTranslation = mdlOffset;
	data.modelRotation = mdlRotOffset;
	InitializeWheel(data);
	return true;
}

void BaseVehicleComponent::InitializeWheelEntity(pragma::BaseWheelComponent *wheel,const WheelData &data)
{
	wheel->SetWheelDirection(GetWheelDirection());
	wheel->SetMaxSuspensionLength(GetMaxSuspensionLength());
	wheel->SetMaxSuspensionCompression(GetMaxSuspensionCompression());
	wheel->SetWheelRadius(GetWheelRadius());
	wheel->SetSuspensionStiffness(GetSuspensionStiffness());
	wheel->SetWheelDampingCompression(GetWheelDampingCompression());
	wheel->SetFrictionSlip(GetFrictionSlip());
	wheel->SetSteeringAngle(GetSteeringAngle());
	wheel->SetRollInfluence(GetRollInfluence());
	wheel->SetModelTranslation(data.modelTranslation);
	wheel->SetModelRotation(data.modelRotation);
}

BaseEntity *BaseVehicleComponent::GetSteeringWheel() {return m_steeringWheel.get();}
Float BaseVehicleComponent::GetMaxEngineForce() const {return m_maxEngineForce;}
Float BaseVehicleComponent::GetMaxReverseEngineForce() const {return m_maxReverseEngineForce;}
Float BaseVehicleComponent::GetMaxBrakeForce() const {return m_maxBrakeForce;}
void BaseVehicleComponent::SetMaxEngineForce(Float force) {m_maxEngineForce = force;}
void BaseVehicleComponent::SetMaxReverseEngineForce(Float force) {m_maxReverseEngineForce = force;}
void BaseVehicleComponent::SetMaxBrakeForce(Float force) {m_maxBrakeForce = force;}
void BaseVehicleComponent::SetAcceleration(Float acc) {m_acceleration = acc;}
void BaseVehicleComponent::SetTurnSpeed(Float speed) {m_turnSpeed = speed;}
void BaseVehicleComponent::SetMaxTurnAngle(Float ang) {m_maxTurnAngle = ang;}

void BaseVehicleComponent::Think(double tDelta)
{
#ifdef ENABLE_DEPRECATED_PHYSICS
	if(m_vhcRaycast == nullptr)
		return;
	m_vhcRaycast->updateVehicle(tDelta);
	if(HasDriver())
	{
		auto *driver = GetDriver();
		if(driver->IsPlayer())
		{
			auto plComponent = driver->GetPlayerComponent();
			auto bResetMovement = true;
			if(plComponent->GetActionInput(Action::MoveForward))
			{
				auto max = GetMaxEngineForce();
				auto acc = GetAcceleration();
				auto force = GetEngineForce();
				if(force < max)
				{
					force = umath::min(force +acc *CFloat(tDelta),max);
					SetEngineForce(force);
				}
				bResetMovement = false;
			}
			if(plComponent->GetActionInput(Action::Jump))
			{
				auto max = GetMaxBrakeForce();
				auto acc = 4000.f;//GetAcceleration();
				auto force = GetBrakeForce();
				if(force < max)
				{
					force = umath::min(force +acc *CFloat(tDelta),max);
					SetBrakeForce(force);
				}
			}
			else
			{
				SetBrakeForce(0.f);
				if(plComponent->GetActionInput(Action::MoveBackward))
				{
					auto min = -GetMaxReverseEngineForce();
					auto acc = GetAcceleration();
					auto force = GetEngineForce();
					if(force > min)
					{
						force = umath::max(force -acc *CFloat(tDelta),min);
						SetEngineForce(force);
					}
					bResetMovement = false;
				}
			}
			if(bResetMovement == true)
			{
				auto acc = GetAcceleration();
				auto force = GetEngineForce();
				force = umath::approach(force,0.f,acc *CFloat(tDelta));
				SetEngineForce(force);
			}
			auto bResetSteering = true;
			if(plComponent->GetActionInput(Action::MoveLeft))
			{
				auto max = GetMaxTurnAngle();
				auto turnSpeed = GetTurnSpeed();
				auto ang = GetSteeringAngle();
				if(ang < max)
				{
					ang = umath::min(ang +turnSpeed *CFloat(tDelta),max);
					SetSteeringAngle(ang);
				}
				bResetSteering = false;
			}
			if(plComponent->GetActionInput(Action::MoveRight))
			{
				auto min = -GetMaxTurnAngle();
				auto turnSpeed = GetTurnSpeed();
				auto ang = GetSteeringAngle();
				if(ang > min)
				{
					ang = umath::max(ang -turnSpeed *CFloat(tDelta),min);
					SetSteeringAngle(ang);
				}
				bResetSteering = false;
			}
			if(bResetSteering == true)
			{
				auto turnSpeed = GetTurnSpeed();
				auto ang = GetSteeringAngle();
				ang = umath::approach(ang,0.f,turnSpeed *CFloat(tDelta));
				SetSteeringAngle(ang);
			}
		}
	}
#endif
}
#ifdef ENABLE_DEPRECATED_PHYSICS
btWheelInfo *BaseVehicleComponent::GetWheelInfo(int wheel)
{
	if(m_vhcRaycast == nullptr)
		return nullptr;
	if(wheel >= m_vhcRaycast->getNumWheels())
		return nullptr;
	return &m_vhcRaycast->getWheelInfo(wheel);
}
#endif
void BaseVehicleComponent::SetEngineForce(Float force)
{
#ifdef ENABLE_DEPRECATED_PHYSICS
	m_engineForce = force;
	if(m_vhcRaycast == nullptr)
		return;
	for(auto i=0;i<m_vhcRaycast->getNumWheels();i++)
		m_vhcRaycast->applyEngineForce(force *PhysEnv::WORLD_SCALE,i);
#endif
}

void BaseVehicleComponent::SetBrakeForce(Float force)
{
#ifdef ENABLE_DEPRECATED_PHYSICS
	m_brakeForce = force;
	if(m_vhcRaycast == nullptr)
		return;
	for(auto i=0;i<m_vhcRaycast->getNumWheels();i++)
		m_vhcRaycast->setBrake(force *PhysEnv::WORLD_SCALE,i);
#endif
}

void BaseVehicleComponent::Initialize()
{
	BaseEntityComponent::Initialize();

	BindEventUnhandled(BasePhysicsComponent::EVENT_ON_PHYSICS_INITIALIZED,[this](std::reference_wrapper<pragma::ComponentEvent> evData) {
		auto pPhysComponent = GetEntity().GetPhysicsComponent();
		if(pPhysComponent.expired())
			return;
		pPhysComponent->AddCollisionFilter(CollisionMask::Vehicle);
		OnPhysicsInitialized();
		});

	auto &ent = GetEntity();
	auto pPhysComponent = ent.GetPhysicsComponent();
	if(pPhysComponent.valid())
		pPhysComponent->SetCollisionFilterGroup(pPhysComponent->GetCollisionFilter() | CollisionMask::Vehicle);
	auto whRenderComponent = ent.AddComponent("render");
	if(whRenderComponent.valid())
		static_cast<BaseRenderComponent*>(whRenderComponent.get())->SetCastShadows(true);
}

unsigned char BaseVehicleComponent::GetWheelCount() {return static_cast<uint8_t>(m_wheels.size());}

Vector3 &BaseVehicleComponent::GetWheelDirection() {return m_wheelInfo.wheelDirection;}
void BaseVehicleComponent::SetWheelDirection(const Vector3 &dir)
{
	m_wheelInfo.wheelDirection = dir;
	for(auto it=m_wheels.begin();it!=m_wheels.end();++it)
	{
		auto &data = *it;
		if(data.hWheel.valid())
			data.hWheel->SetWheelDirection(dir);
	}
}
Float BaseVehicleComponent::GetMaxSuspensionLength() const {return m_wheelInfo.suspensionLength;}
void BaseVehicleComponent::SetMaxSuspensionLength(Float len)
{
	m_wheelInfo.suspensionLength = len;
	for(auto it=m_wheels.begin();it!=m_wheels.end();++it)
	{
		auto &data = *it;
		if(data.hWheel.valid())
			data.hWheel->SetMaxSuspensionLength(len);
	}
}
Float BaseVehicleComponent::GetMaxSuspensionCompression() const {return m_wheelInfo.suspensionCompression;}
void BaseVehicleComponent::SetMaxSuspensionCompression(Float cmp)
{
	m_wheelInfo.suspensionCompression = cmp;
	for(auto it=m_wheels.begin();it!=m_wheels.end();++it)
	{
		auto &data = *it;
		if(data.hWheel.valid())
			data.hWheel->SetMaxSuspensionCompression(cmp);
	}
}
Float BaseVehicleComponent::GetMaxDampingRelaxation() const {return m_wheelInfo.dampingRelaxation;}
void BaseVehicleComponent::SetMaxDampingRelaxation(Float damping)
{
	m_wheelInfo.dampingRelaxation = damping;
	for(auto it=m_wheels.begin();it!=m_wheels.end();++it)
	{
		auto &data = *it;
		if(data.hWheel.valid())
			data.hWheel->SetMaxDampingRelaxation(damping);
	}
}
Float BaseVehicleComponent::GetWheelRadius() const {return m_wheelInfo.wheelRadius;}
void BaseVehicleComponent::SetWheelRadius(Float radius)
{
	m_wheelInfo.wheelRadius = radius;
	for(auto it=m_wheels.begin();it!=m_wheels.end();++it)
	{
		auto &data = *it;
		if(data.hWheel.valid())
			data.hWheel->SetWheelRadius(radius);
	}
}
Float BaseVehicleComponent::GetSuspensionStiffness() const {return m_wheelInfo.suspensionStiffness;}
void BaseVehicleComponent::SetSuspensionStiffness(Float stiffness)
{
	m_wheelInfo.suspensionStiffness = stiffness;
	for(auto it=m_wheels.begin();it!=m_wheels.end();++it)
	{
		auto &data = *it;
		if(data.hWheel.valid())
			data.hWheel->SetSuspensionStiffness(stiffness);
	}
}
Float BaseVehicleComponent::GetWheelDampingCompression() const {return m_wheelInfo.wheelDampingCompression;}
void BaseVehicleComponent::SetWheelDampingCompression(Float cmp)
{
	m_wheelInfo.wheelDampingCompression = cmp;
	for(auto it=m_wheels.begin();it!=m_wheels.end();++it)
	{
		auto &data = *it;
		if(data.hWheel.valid())
			data.hWheel->SetWheelDampingCompression(cmp);
	}
}
Float BaseVehicleComponent::GetFrictionSlip() const {return m_wheelInfo.frictionSlip;}
void BaseVehicleComponent::SetFrictionSlip(Float slip)
{
	m_wheelInfo.frictionSlip = slip;
	for(auto it=m_wheels.begin();it!=m_wheels.end();++it)
	{
		auto &data = *it;
		if(data.hWheel.valid())
			data.hWheel->SetFrictionSlip(slip);
	}
}
Float BaseVehicleComponent::GetSteeringAngle() const {return m_wheelInfo.steeringAngle;}
void BaseVehicleComponent::SetSteeringAngle(Float ang)
{
	m_wheelInfo.steeringAngle = ang;
	for(auto it=m_wheels.begin();it!=m_wheels.end();++it)
	{
		auto &data = *it;
		if(data.hWheel.valid())
		{
			auto &wheel = data.hWheel;
			if(wheel->IsFrontWheel())
				wheel->SetSteeringAngle(ang);
		}
	}
}
Float BaseVehicleComponent::GetRollInfluence() const {return m_wheelInfo.rollInfluence;}
void BaseVehicleComponent::SetRollInfluence(Float influence)
{
	m_wheelInfo.rollInfluence = influence;
	for(auto it=m_wheels.begin();it!=m_wheels.end();++it)
	{
		auto &data = *it;
		if(data.hWheel.valid())
			data.hWheel->SetRollInfluence(influence);
	}
}

#endif

#endif
