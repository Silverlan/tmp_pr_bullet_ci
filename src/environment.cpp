/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "environment.hpp"
#include "collision_object.hpp"
#include "constraint.hpp"
#include "controller.hpp"
#include "shape.hpp"
#include "material.hpp"
#include "overlapfiltercallback.hpp"
#include "debug.hpp"
#include "kinematic_character_controller.hpp"
#include <BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <BulletSoftBody/btDefaultSoftBodySolver.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletCollision/BroadphaseCollision/btAxisSweep3.h>
#include <pragma/networkstate/networkstate.h>
#include <pragma/game/game.h>
#include <pragma/util/util_game.hpp>
#include <pragma/physics/physsoftbodyinfo.hpp>
#include <pragma/physics/raytraces.h>
#include <pragma/physics/raycast_filter.hpp>
#include <pragma/entities/components/base_physics_component.hpp>
#include <pragma/entities/trigger/base_trigger_touch.hpp>
#include <pragma/entities/baseentity.h>
#include <pragma/audio/alsound.h>
#include <pragma/audio/alsound_type.h>
#include <unordered_set>

enum class BulletBroadphaseType : uint32_t
{
	Dbvt = 0u, // Should be more efficient than AxisSweep3_32Bit, but causes massive performance drop for some unknown reason
	AxisSweep3_32Bit
};

const double pragma::physics::BtEnvironment::WORLD_SCALE = util::pragma::units_to_metres(1.0);
const double pragma::physics::BtEnvironment::WORLD_SCALE_SQR = umath::pow(BtEnvironment::WORLD_SCALE,2.0);
const float pragma::physics::BtEnvironment::CCD_MOTION_THRESHOLD = 4.f *static_cast<float>(WORLD_SCALE);
const float pragma::physics::BtEnvironment::CCD_SWEPT_SPHERE_RADIUS = 2.f *static_cast<float>(WORLD_SCALE);
static const float PHYS_CONSTRAINT_DEBUG_DRAW_SIZE = 100.f;
static const auto PHYS_BULLET_BROADPHASE_TYPE = BulletBroadphaseType::AxisSweep3_32Bit;

pragma::physics::BtEnvironment *g_simEnvironment = nullptr;

class PhysBulletWorld
	: public btWorldType
{
public:
	using btWorldType::btWorldType;
	virtual void updateAabbs() override
	{
		BT_PROFILE("updateAabbs");

		btTransform predictedTrans;
		for ( int i=0;i<m_collisionObjects.size();i++)
		{
			btCollisionObject* colObj = m_collisionObjects[i];
			btAssert(colObj->getWorldArrayIndex() == i);

			//only update aabb of active objects
			if (m_forceUpdateAllAabbs || colObj->isActive())
			{
				updateSingleAabb(colObj);
			}
			else
			{
				auto *c = static_cast<pragma::physics::ICollisionObject*>(colObj->getUserPointer());
				if(c != nullptr && c->ShouldUpdateAABB())
				{
					updateSingleAabb(colObj);
					c->ResetUpdateAABBFlag();
				}
			}
		}
	};
};

extern void btInitCustomMaterialCombinerCallback();
pragma::physics::BtEnvironment::BtEnvironment(NetworkState &state)
	: pragma::physics::IEnvironment{state}
{
	btInitCustomMaterialCombinerCallback();

	if(std::is_same<btWorldType,btSoftRigidDynamicsWorld>::value == true)
		m_btCollisionConfiguration = std::make_unique<btSoftBodyRigidBodyCollisionConfiguration>();
	else
		m_btCollisionConfiguration = std::make_unique<btDefaultCollisionConfiguration>();
	m_btDispatcher = std::make_unique<btCollisionDispatcher>(m_btCollisionConfiguration.get());

	switch(PHYS_BULLET_BROADPHASE_TYPE)
	{
	case BulletBroadphaseType::Dbvt:
		m_btOverlappingPairCache = std::make_unique<btDbvtBroadphase>();
		break;
	case BulletBroadphaseType::AxisSweep3_32Bit:
	{
		auto min = uvec::create_bt(Vector3{-16'384.f,-16'384.f,-16'384.f});
		auto max = uvec::create_bt(Vector3{16'384.f,16'384.f,16'384.f});
		m_btOverlappingPairCache = std::make_unique<bt32BitAxisSweep3>(min,max);
		break;
	}
	}

	m_btGhostPairCallback = std::make_unique<btGhostPairCallback>();
	m_btOverlappingPairCache->getOverlappingPairCache()->setInternalGhostPairCallback(m_btGhostPairCallback.get());
	m_overlapFilterCallback = std::make_unique<PhysOverlapFilterCallback>();
	m_btOverlappingPairCache->getOverlappingPairCache()->setOverlapFilterCallback(m_overlapFilterCallback.get());
	m_btSolver = std::make_unique<btSequentialImpulseConstraintSolver>();
	m_softBodySolver = std::unique_ptr<btSoftBodySolver>(new btDefaultSoftBodySolver);

	m_btWorld = std::make_unique<PhysBulletWorld>(m_btDispatcher.get(),m_btOverlappingPairCache.get(),m_btSolver.get(),m_btCollisionConfiguration.get(),m_softBodySolver.get());
	m_btWorld->setGravity(btVector3(0.f,0.f,0.f));
	m_btWorld->setInternalTickCallback(&BtEnvironment::SimulationCallback,this);
	m_btWorld->setForceUpdateAllAabbs(false);

	m_softBodyWorldInfo = std::make_unique<btSoftBodyWorldInfo>();
	m_softBodyWorldInfo->m_broadphase = m_btOverlappingPairCache.get();
	m_softBodyWorldInfo->m_dispatcher = m_btDispatcher.get();
	m_softBodyWorldInfo->m_gravity = m_btWorld->getGravity();
	m_softBodyWorldInfo->m_sparsesdf.Initialize();
}
pragma::physics::BtEnvironment::~BtEnvironment() {}
umath::Transform pragma::physics::BtEnvironment::CreateTransform(const btTransform &btTransform)
{
	return umath::Transform {ToPragmaPosition(btTransform.getOrigin()),uquat::create(btTransform.getRotation())};
}
btTransform pragma::physics::BtEnvironment::CreateBtTransform(const umath::Transform &t)
{
	return btTransform {uquat::create_bt(t.GetRotation()),ToBtPosition(t.GetOrigin())};
}
Vector3 pragma::physics::BtEnvironment::ToPragmaPosition(const btVector3 &pos) {return Vector3{pos.x(),pos.y(),pos.z()} /static_cast<float>(WORLD_SCALE);}
Color pragma::physics::BtEnvironment::ToPragmaColor(const btVector3 &col) {return Color{static_cast<int16_t>(col.x() *255.f),static_cast<int16_t>(col.y() *255.f),static_cast<int16_t>(col.z() *255.f)};}
btVector3 pragma::physics::BtEnvironment::ToBtPosition(const Vector3 &pos) {return btVector3{pos.x,pos.y,pos.z} *static_cast<float>(WORLD_SCALE);}
Vector3 pragma::physics::BtEnvironment::ToPragmaNormal(const btVector3 &n) {return Vector3{static_cast<float>(n.x()),static_cast<float>(n.y()),static_cast<float>(n.z())};}
btVector3 pragma::physics::BtEnvironment::ToBtNormal(const Vector3 &n) {return btVector3{n.x,n.y,n.z};}
double pragma::physics::BtEnvironment::ToPragmaDistance(btScalar d) {return d /WORLD_SCALE;}
btScalar pragma::physics::BtEnvironment::ToBtDistance(double d) {return d *WORLD_SCALE;}
std::shared_ptr<pragma::physics::IMaterial> pragma::physics::BtEnvironment::CreateMaterial(float staticFriction,float dynamicFriction,float restitution)
{
	return CreateSharedPtr<BtMaterial>(*this,staticFriction,dynamicFriction,restitution);
}
util::TSharedHandle<pragma::physics::ICollisionObject> pragma::physics::BtEnvironment::CreatePlane(const Vector3 &n,float d,const IMaterial &mat)
{
	// TODO
	return nullptr;
}
util::TSharedHandle<pragma::physics::IVehicle> pragma::physics::BtEnvironment::CreateVehicle(const VehicleCreateInfo &vhcDesc)
{
	// TODO
	return nullptr;
}
void pragma::physics::BtEnvironment::UpdateSurfaceTypes()
{
	// TODO
}
// TODO
/*pragma::physics::IVisualDebugger *pragma::physics::BtEnvironment::InitializeVisualDebugger()
{
	if(m_visualDebugger)
		return m_visualDebugger.get();
	auto visDebugger = std::make_shared<BtVisualDebugger>();
	m_visualDebugger = visDebugger;
	auto *world = GetWorld();
	world->setDebugDrawer(visDebugger.get());
	return m_visualDebugger.get();
}*/
pragma::physics::BtRigidBody &pragma::physics::BtEnvironment::ToBtType(IRigidBody &body) {return dynamic_cast<BtRigidBody&>(body);}
float pragma::physics::BtEnvironment::GetWorldScale() const {return WORLD_SCALE;}

namespace pragma::physics
{
	class BtDebugDrawer
		: public btIDebugDraw
	{
	public:
		BtDebugDrawer(pragma::physics::BtEnvironment &env,pragma::physics::IVisualDebugger &debugger)
			: m_env{env},m_debugger{debugger}
		{}
		virtual ~BtDebugDrawer() override {}
		virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& fromColor, const btVector3& toColor) override
		{
			m_debugger.DrawLine(
				pragma::physics::BtEnvironment::ToPragmaPosition(from),pragma::physics::BtEnvironment::ToPragmaPosition(to),
				pragma::physics::BtEnvironment::ToPragmaColor(fromColor),pragma::physics::BtEnvironment::ToPragmaColor(toColor)
			);
		}
		virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& color) override
		{
			m_debugger.DrawLine(
				pragma::physics::BtEnvironment::ToPragmaPosition(from),pragma::physics::BtEnvironment::ToPragmaPosition(to),
				pragma::physics::BtEnvironment::ToPragmaColor(color)
			);
		}
		virtual void drawSphere(const btVector3& p, btScalar radius, const btVector3& color) override
		{
			// TODO
		}
		virtual void drawTriangle(const btVector3& a, const btVector3& b, const btVector3& c, const btVector3& color, btScalar alpha) override
		{
			auto col = pragma::physics::BtEnvironment::ToPragmaColor(color);
			col.a = alpha *255.f;
			m_debugger.DrawTriangle(
				pragma::physics::BtEnvironment::ToPragmaPosition(a),pragma::physics::BtEnvironment::ToPragmaPosition(b),pragma::physics::BtEnvironment::ToPragmaPosition(c),
				col,col,col
			);
		}
		virtual void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color) override
		{
			m_debugger.DrawPoint(pragma::physics::BtEnvironment::ToPragmaPosition(PointOnB),pragma::physics::BtEnvironment::ToPragmaColor(color));
		}
		virtual void reportErrorWarning(const char* warningString) override
		{
			m_debugger.ReportErrorWarning(warningString);
		}
		virtual void draw3dText(const btVector3& location, const char* textString) override
		{
			m_debugger.DrawText(textString,pragma::physics::BtEnvironment::ToPragmaPosition(location),Color::White,1.f);
		}
		virtual void setDebugMode(int debugMode) override {m_debugMode = debugMode;}
		virtual int getDebugMode() const override { return m_debugMode; }
	private:
		pragma::physics::BtEnvironment &m_env;
		pragma::physics::IVisualDebugger &m_debugger;
		int m_debugMode = 0;
	};
};
void pragma::physics::BtEnvironment::OnVisualDebuggerChanged(pragma::physics::IVisualDebugger *debugger)
{
	if(!debugger)
	{
		m_btWorld->setDebugDrawer(nullptr);
		m_btDebugDrawer = nullptr;
		return;
	}
	m_btDebugDrawer = std::make_unique<BtDebugDrawer>(*this,*debugger);
	m_btWorld->setDebugDrawer(m_btDebugDrawer.get());
	m_btDebugDrawer->setDebugMode(
		btIDebugDraw::DebugDrawModes::DBG_DrawWireframe | btIDebugDraw::DebugDrawModes::DBG_DrawAabb | btIDebugDraw::DebugDrawModes::DBG_DrawConstraints
	);
}

#ifdef BT_ENABLE_PROFILE
static void bt_profile_manager_dump_recursive(CProfileIterator* profileIterator, int spacing)
{
	profileIterator->First();
	if (profileIterator->Is_Done())
		return;

	float accumulated_time = 0, parent_time = profileIterator->Is_Root() ? CProfileManager::Get_Time_Since_Reset() : profileIterator->Get_Current_Parent_Total_Time();
	int i;
	int frames_since_reset = CProfileManager::Get_Frame_Count_Since_Reset();
	for (i = 0; i < spacing; i++) Con::cout<<".";
	Con::cout<<"----------------------------------\n";
	for (i = 0; i < spacing; i++) Con::cout<<".";
	Con::cout<<"Profiling: "<<profileIterator->Get_Current_Parent_Name()<<" (total running time: "<<parent_time<<" ms) ---\n";
	float totalTime = 0.f;

	int numChildren = 0;

	for (i = 0; !profileIterator->Is_Done(); i++, profileIterator->Next())
	{
		numChildren++;
		float current_total_time = profileIterator->Get_Current_Total_Time();
		accumulated_time += current_total_time;
		float fraction = parent_time > SIMD_EPSILON ? (current_total_time / parent_time) * 100 : 0.f;
		{
			int i;
			for (i = 0; i < spacing; i++) Con::cout<<".";
		}
		Con::cout<<i<<" -- "<<profileIterator->Get_Current_Name()<<" ("<<fraction<<" %%) :: "<<(current_total_time / (double)frames_since_reset)<<" ms / frame ("<<profileIterator->Get_Current_Total_Calls()<<" calls)\n";
		totalTime += current_total_time;
		//recurse into children
	}

	if (parent_time < accumulated_time)
	{
		//printf("what's wrong\n");
	}
	for (i = 0; i < spacing; i++) Con::cout<<".";
	Con::cout<<"Unaccounted: ("<<(parent_time > SIMD_EPSILON ? ((parent_time - accumulated_time) / parent_time) * 100 : 0.f)<<" %%) :: "<<(parent_time - accumulated_time)<<" ms\n";

	for (i = 0; i < numChildren; i++)
	{
		profileIterator->Enter_Child(i);
		bt_profile_manager_dump_recursive(profileIterator, spacing + 3);
		profileIterator->Enter_Parent();
	}
}

static void bt_profile_manager_dump_all()
{
	CProfileIterator* profileIterator = 0;
	profileIterator = CProfileManager::Get_Iterator();

	bt_profile_manager_dump_recursive(profileIterator, 0);

	CProfileManager::Release_Iterator(profileIterator);
}
#endif

void pragma::physics::BtEnvironment::StartProfiling()
{
#ifdef BT_ENABLE_PROFILE
	btSetCustomEnterProfileZoneFunc(CProfileManager::Start_Profile);
	btSetCustomLeaveProfileZoneFunc(CProfileManager::Stop_Profile);
#endif
}
void pragma::physics::BtEnvironment::EndProfiling()
{
#ifdef BT_ENABLE_PROFILE
	Con::cout<<"-------- Physics Profiler Results --------"<<Con::endl;
	// CProfileManager::dumpAll(); // dumpAll uses printf, which we can't use
	bt_profile_manager_dump_all(); // Same as CProfileManager::dumpAll, but uses Con::cout instead of printf
	Con::cout<<"------------------------------------------"<<Con::endl;
#endif
}
void pragma::physics::BtEnvironment::SimulationCallback(btDynamicsWorld *world,btScalar timeStep)
{
	void *userData = world->getWorldUserInfo();
	auto *env = static_cast<BtEnvironment*>(userData);
	env->SimulationCallback(timeStep);
}
btSoftBodySolver &pragma::physics::BtEnvironment::GetSoftBodySolver() {return *m_softBodySolver;}
const btSoftBodySolver &pragma::physics::BtEnvironment::GetSoftBodySolver() const {return const_cast<BtEnvironment*>(this)->GetSoftBodySolver();}
btWorldType *pragma::physics::BtEnvironment::GetWorld() {return m_btWorld.get();}
btDefaultCollisionConfiguration *pragma::physics::BtEnvironment::GetBtCollisionConfiguration() {return m_btCollisionConfiguration.get();}
btCollisionDispatcher *pragma::physics::BtEnvironment::GetBtCollisionDispatcher() {return m_btDispatcher.get();}
btBroadphaseInterface *pragma::physics::BtEnvironment::GetBtOverlappingPairCache() {return m_btOverlappingPairCache.get();}
btSequentialImpulseConstraintSolver *pragma::physics::BtEnvironment::GetBtConstraintSolver() {return m_btSolver.get();}
btSoftBodyWorldInfo *pragma::physics::BtEnvironment::GetBtSoftBodyWorldInfo() {return m_softBodyWorldInfo.get();}
util::TSharedHandle<pragma::physics::IFixedConstraint> pragma::physics::BtEnvironment::AddFixedConstraint(std::unique_ptr<btFixedConstraint> c)
{
	c->setDbgDrawSize(PHYS_CONSTRAINT_DEBUG_DRAW_SIZE);
	auto constraint = CreateSharedHandle<BtFixedConstraint>(*this,std::move(c));
	AddConstraint(*constraint);
	return util::shared_handle_cast<BtFixedConstraint,IFixedConstraint>(constraint);
}
util::TSharedHandle<pragma::physics::IBallSocketConstraint> pragma::physics::BtEnvironment::AddBallSocketConstraint(std::unique_ptr<btPoint2PointConstraint> c)
{
	c->setDbgDrawSize(PHYS_CONSTRAINT_DEBUG_DRAW_SIZE);
	auto constraint = CreateSharedHandle<BtBallSocketConstraint>(*this,std::move(c));
	AddConstraint(*constraint);
	return util::shared_handle_cast<BtBallSocketConstraint,IBallSocketConstraint>(constraint);
}
util::TSharedHandle<pragma::physics::IHingeConstraint> pragma::physics::BtEnvironment::AddHingeConstraint(std::unique_ptr<btHingeConstraint> c)
{
	c->setDbgDrawSize(PHYS_CONSTRAINT_DEBUG_DRAW_SIZE);
	auto constraint = CreateSharedHandle<BtHingeConstraint>(*this,std::move(c));
	AddConstraint(*constraint);
	return util::shared_handle_cast<BtHingeConstraint,IHingeConstraint>(constraint);
}
util::TSharedHandle<pragma::physics::ISliderConstraint> pragma::physics::BtEnvironment::AddSliderConstraint(std::unique_ptr<btSliderConstraint> c)
{
	// TODO
	/*c->setDbgDrawSize(PHYS_CONSTRAINT_DEBUG_DRAW_SIZE);
	auto constraint = CreateSharedHandle<BtSliderConstraint>(*this,std::move(c));
	AddConstraint(*constraint);
	return util::shared_handle_cast<BtSliderConstraint,ISliderConstraint>(constraint);

	auto *rigidBody0 = hBody.Get();
	auto *rigidBody1 = bodySrc;
	auto pivot0 = Vector3{};
	auto pivot1 = rigidBody0->GetPos();
	auto rotation0 = -rot *uquat::create(EulerAngles(0,90,0));
	auto rotation1 = rotation0;
	rotation0 = tgt->GetTransformComponent()->GetOrientation() *rotation0;

	auto slider = physEnv->CreateDoFSpringConstraint(*rigidBody0,pivot0,rotation0,*rigidBody1,pivot1,rotation1);
	if(slider != nullptr)
	{
		slider->SetEntity(GetEntity());
		slider->SetLimit(pragma::physics::IDoFSpringConstraint::AxisType::Linear,pragma::Axis::X,0.f,l);
		slider->SetLimit(pragma::physics::IDoFSpringConstraint::AxisType::Linear,pragma::Axis::Y,0.f,0.f);
		slider->SetLimit(pragma::physics::IDoFSpringConstraint::AxisType::Linear,pragma::Axis::Z,0.f,0.f);
		slider->SetLimit(pragma::physics::IDoFSpringConstraint::AxisType::Angular,pragma::Axis::X,0.f,0.f);
		slider->SetLimit(pragma::physics::IDoFSpringConstraint::AxisType::Angular,pragma::Axis::Y,0.f,0.f);
		slider->SetLimit(pragma::physics::IDoFSpringConstraint::AxisType::Angular,pragma::Axis::Z,0.f,0.f);
		m_constraints.push_back(util::shared_handle_cast<pragma::physics::IDoFSpringConstraint,pragma::physics::IConstraint>(slider));
	}
	*/
	return nullptr;
}
util::TSharedHandle<pragma::physics::IConeTwistConstraint> pragma::physics::BtEnvironment::AddConeTwistConstraint(std::unique_ptr<btConeTwistConstraint> c)
{
	c->setDbgDrawSize(PHYS_CONSTRAINT_DEBUG_DRAW_SIZE);
	auto constraint = CreateSharedHandle<BtConeTwistConstraint>(*this,std::move(c));
	AddConstraint(*constraint);
	return util::shared_handle_cast<BtConeTwistConstraint,IConeTwistConstraint>(constraint);
}
util::TSharedHandle<pragma::physics::IDoFConstraint> pragma::physics::BtEnvironment::AddDoFConstraint(std::unique_ptr<btGeneric6DofConstraint> c)
{
	c->setDbgDrawSize(PHYS_CONSTRAINT_DEBUG_DRAW_SIZE);
	auto constraint = CreateSharedHandle<BtDoFConstraint>(*this,std::move(c));
	AddConstraint(*constraint);
	return util::shared_handle_cast<BtDoFConstraint,IDoFConstraint>(constraint);
}
util::TSharedHandle<pragma::physics::IDoFSpringConstraint> pragma::physics::BtEnvironment::AddDoFSpringConstraint(std::unique_ptr<btGeneric6DofSpring2Constraint> c)
{
	c->setDbgDrawSize(PHYS_CONSTRAINT_DEBUG_DRAW_SIZE);
	auto constraint = CreateSharedHandle<BtDoFSpringConstraint>(*this,std::move(c));
	AddConstraint(*constraint);
	return util::shared_handle_cast<BtDoFSpringConstraint,IDoFSpringConstraint>(constraint);
}
util::TSharedHandle<pragma::physics::IFixedConstraint> pragma::physics::BtEnvironment::CreateFixedConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB)
{
	auto &bodyA = ToBtType(a).GetInternalObject();
	auto &bodyB = ToBtType(b).GetInternalObject();

	btTransform transformA;
	transformA.setIdentity();
	transformA.setOrigin(btVector3(pivotA.x,pivotA.y,pivotA.z) *WORLD_SCALE);
	transformA.setRotation(btQuaternion(rotA.x,rotA.y,rotA.z,rotA.w));

	btTransform transformB;
	transformB.setIdentity();
	transformB.setOrigin(btVector3(pivotB.x,pivotB.y,pivotB.z) *WORLD_SCALE);
	transformB.setRotation(btQuaternion(rotB.x,rotB.y,rotB.z,rotB.w));
	return AddFixedConstraint(std::make_unique<btFixedConstraint>(bodyA,bodyB,transformA,transformB));
}
util::TSharedHandle<pragma::physics::IBallSocketConstraint> pragma::physics::BtEnvironment::CreateBallSocketConstraint(IRigidBody &a,const Vector3 &pivotA,IRigidBody &b,const Vector3 &pivotB)
{
	auto &bodyA = ToBtType(a).GetInternalObject();
	auto &bodyB = ToBtType(b).GetInternalObject();
	return AddBallSocketConstraint(std::make_unique<btPoint2PointConstraint>(bodyA,bodyB,btVector3(pivotA.x,pivotA.y,pivotA.z) *WORLD_SCALE,btVector3(pivotB.x,pivotB.y,pivotB.z) *WORLD_SCALE));
}
util::TSharedHandle<pragma::physics::IHingeConstraint> pragma::physics::BtEnvironment::CreateHingeConstraint(IRigidBody &a,const Vector3 &pivotA,IRigidBody &b,const Vector3 &pivotB,const Vector3 &axis)
{
	auto &bodyA = ToBtType(a).GetInternalObject();
	auto &bodyB = ToBtType(b).GetInternalObject();
	btVector3 btAxis(axis.x,axis.y,axis.z);
	return AddHingeConstraint(std::make_unique<btHingeConstraint>(bodyA,bodyB,btVector3(pivotA.x,pivotA.y,pivotA.z) *WORLD_SCALE,btVector3(pivotB.x,pivotB.y,pivotB.z) *WORLD_SCALE,btAxis,btAxis));
}
util::TSharedHandle<pragma::physics::ISliderConstraint> pragma::physics::BtEnvironment::CreateSliderConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat&,IRigidBody &b,const Vector3 &pivotB,const Quat&)
{
	auto &bodyA = ToBtType(a).GetInternalObject();
	auto &bodyB = ToBtType(b).GetInternalObject();
	btTransform btTransformA;
	btTransformA.setIdentity();
	btTransformA.setOrigin(btVector3(pivotA.x,pivotA.y,pivotA.z) *WORLD_SCALE);
	//	btTransformA.setRotation(btQuaternion(0,0,0,1));
	//	btTransformA.getBasis().setEulerYPR(pivotB.x,pivotB.y,pivotB.z);

	//btTransformA.setOrigin(btVector3(pivotA.x,pivotA.y,pivotA.z) *WORLD_SCALE);
	//btTransformA.setRotation(btQuaternion(rotA.x,rotA.y,rotA.z,rotA.w));
	btTransform btTransformB;
	btTransformB.setIdentity();
	btTransformB.setOrigin(btVector3(pivotB.x,pivotB.y,pivotB.z) *WORLD_SCALE);
	//btTransformB.setRotation(btQuaternion(0,0,0,1));


	//btTransformB.setOrigin(btVector3(pivotB.x,pivotB.y,pivotB.z) *WORLD_SCALE);
	//btTransformB.setRotation(btQuaternion(rotB.x,rotB.y,rotB.z,rotB.w));
	//btTransformA = btTransform::getIdentity();
	//btTransformB = btTransform::getIdentity();
	return AddSliderConstraint(std::make_unique<btSliderConstraint>(bodyA,bodyB,btTransformA,btTransformB,true));
}
util::TSharedHandle<pragma::physics::IConeTwistConstraint> pragma::physics::BtEnvironment::CreateConeTwistConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB)
{
	auto v = 1.f /sqrtf(2.f); // umath::cos(M_PI_4);
	auto rotSrc = rotA;
	auto rotOffset = Quat(v,0.f,-v,0.f);
	rotSrc = rotSrc *rotOffset;
	auto rotTgt = rotB;
	rotTgt = rotTgt *rotSrc;

	auto &bodyA = ToBtType(a).GetInternalObject();
	auto &bodyB = ToBtType(b).GetInternalObject();
	btTransform btTransformA;
	btTransformA.setIdentity();
	btTransformA.setOrigin(btVector3(pivotA.x,pivotA.y,pivotA.z) *WORLD_SCALE);
	btTransformA.setRotation(btQuaternion(rotSrc.x,rotSrc.y,rotSrc.z,rotSrc.w));
	btTransform btTransformB;
	btTransformB.setIdentity();
	btTransformB.setOrigin(btVector3(pivotB.x,pivotB.y,pivotB.z) *WORLD_SCALE);
	btTransformB.setRotation(btQuaternion(rotTgt.x,rotTgt.y,rotTgt.z,rotTgt.w));
	return AddConeTwistConstraint(std::make_unique<btConeTwistConstraint>(bodyA,bodyB,btTransformA,btTransformB));
}
util::TSharedHandle<pragma::physics::IDoFConstraint> pragma::physics::BtEnvironment::CreateDoFConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB)
{
	auto &bodyA = ToBtType(a).GetInternalObject();
	auto &bodyB = ToBtType(b).GetInternalObject();

	btTransform tA;
	tA.setIdentity();
	tA.setOrigin(btVector3(pivotA.x,pivotA.y,pivotA.z) *WORLD_SCALE);
	tA.setRotation(btQuaternion(rotA.x,rotA.y,rotA.z,rotA.w));
	btTransform tB;
	tB.setIdentity();
	tB.setOrigin(btVector3(pivotB.x,pivotB.y,pivotB.z) *WORLD_SCALE);
	tB.setRotation(btQuaternion(rotB.x,rotB.y,rotB.z,rotB.w));
	return AddDoFConstraint(std::make_unique<btGeneric6DofConstraint>(bodyA,bodyB,tA,tB,true));
}
util::TSharedHandle<pragma::physics::IDoFSpringConstraint> pragma::physics::BtEnvironment::CreateDoFSpringConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB)
{
	auto &bodyA = ToBtType(a).GetInternalObject();
	auto &bodyB = ToBtType(b).GetInternalObject();

	btTransform tA;
	tA.setIdentity();
	tA.setOrigin(btVector3(pivotA.x,pivotA.y,pivotA.z) *WORLD_SCALE);
	tA.setRotation(btQuaternion(rotA.x,rotA.y,rotA.z,rotA.w));
	btTransform tB;
	tB.setIdentity();
	tB.setOrigin(btVector3(pivotB.x,pivotB.y,pivotB.z) *WORLD_SCALE);
	tB.setRotation(btQuaternion(rotB.x,rotB.y,rotB.z,rotB.w));

	return AddDoFSpringConstraint(std::make_unique<btGeneric6DofSpring2Constraint>(bodyA,bodyB,tA,tB));
}
std::shared_ptr<pragma::physics::IConvexShape> pragma::physics::BtEnvironment::CreateCapsuleShape(float halfWidth,float halfHeight,const IMaterial &mat)
{
	auto height = halfHeight *2.f -halfWidth *2.f;
	return CreateSharedPtr<BtConvexShape>(
		*this,
		std::make_shared<btCapsuleShape>(halfWidth *pragma::physics::BtEnvironment::WORLD_SCALE,height *pragma::physics::BtEnvironment::WORLD_SCALE)
	);
}
std::shared_ptr<pragma::physics::IConvexShape> pragma::physics::BtEnvironment::CreateBoxShape(const Vector3 &halfExtents,const IMaterial &mat)
{
	return CreateSharedPtr<BtConvexShape>(
		*this,
		std::make_shared<btBoxShape>(btVector3(halfExtents.x,halfExtents.y,halfExtents.z) *pragma::physics::BtEnvironment::WORLD_SCALE)
	);
}
std::shared_ptr<pragma::physics::IConvexShape> pragma::physics::BtEnvironment::CreateCylinderShape(float radius,float height,const IMaterial &mat)
{
	return CreateSharedPtr<BtConvexShape>(
		*this,
		std::make_shared<btCylinderShape>(btVector3(radius,height,radius) *pragma::physics::BtEnvironment::WORLD_SCALE)
	);
}
std::shared_ptr<pragma::physics::IConvexShape> pragma::physics::BtEnvironment::CreateSphereShape(float radius,const IMaterial &mat)
{
	return CreateSharedPtr<BtConvexShape>(
		*this,
		std::make_shared<btSphereShape>(radius *pragma::physics::BtEnvironment::WORLD_SCALE)
	);
}
std::shared_ptr<pragma::physics::ICompoundShape> pragma::physics::BtEnvironment::CreateTorusShape(uint32_t subdivisions,double outerRadius,double innerRadius,const IMaterial &mat)
{
	// Source: http://bulletphysics.org/Bullet/phpBB3/viewtopic.php?t=7228#p24758
	btVector3 forward(btScalar(0.0),btScalar(1.0),btScalar(0.0));
	btVector3 side(btScalar(outerRadius),btScalar(0.0),btScalar(0.0));

	auto gap = sqrt(2.0 *innerRadius *innerRadius -2.0 *innerRadius *innerRadius* cos((2.0 *SIMD_PI) /static_cast<double>(subdivisions)));

	std::vector<std::shared_ptr<pragma::physics::IShape>> ptrShapes {};
	std::vector<pragma::physics::IShape*> shapes {};
	shapes.reserve(subdivisions);
	ptrShapes.reserve(subdivisions);
	for(auto i=decltype(subdivisions){0};i<subdivisions;++i)
	{
		auto angle = btScalar((static_cast<double>(i) *2.0 *SIMD_PI) /static_cast<double>(subdivisions));
		auto position = side.rotate(forward,angle);
		btQuaternion q(forward,angle);

		auto cylShape = CreateSharedPtr<pragma::physics::BtConvexShape>(*this,std::make_shared<btCylinderShapeZ>(
			btVector3(btScalar(innerRadius),btScalar(innerRadius),btScalar((SIMD_PI /static_cast<double>(subdivisions)) +0.5 *gap))
		));
		cylShape->SetLocalPose(umath::Transform{uvec::create(position),uquat::create(q)});

		auto pShape = std::static_pointer_cast<IShape>(cylShape);
		ptrShapes.push_back(pShape);
		shapes.push_back(pShape.get());
	}
	return CreateCompoundShape(shapes);
}
std::shared_ptr<pragma::physics::IConvexHullShape> pragma::physics::BtEnvironment::CreateConvexHullShape(const IMaterial &mat)
{
	return CreateSharedPtr<BtConvexHullShape>(*this,std::make_shared<btConvexHullShape>());
}
std::shared_ptr<pragma::physics::ITriangleShape> pragma::physics::BtEnvironment::CreateTriangleShape(const IMaterial &mat)
{
	return CreateSharedPtr<BtTriangleShape>(*this);
}
std::shared_ptr<pragma::physics::ICompoundShape> pragma::physics::BtEnvironment::CreateCompoundShape(std::vector<IShape*> &shapes)
{
	// TODO
	return CreateSharedPtr<BtCompoundShape>(*this);
	//return CreateSharedPtr<BtCompoundShape>(*this,std::make_shared<btCompoundShape>(),shapes);
}
std::shared_ptr<pragma::physics::IShape> pragma::physics::BtEnvironment::CreateHeightfieldTerrainShape(uint32_t width,uint32_t length,Scalar maxHeight,uint32_t upAxis,const IMaterial &mat)
{
	std::vector<Vector3> data {}; // TODO: Pass data as argument
	data.resize(width *length);
	auto btShape = std::make_shared<btHeightfieldTerrainShape>(width,length,data.data(),1.f,-maxHeight,maxHeight,upAxis,PHY_ScalarType::PHY_FLOAT,false);
	return CreateSharedPtr<BtHeightfield>(*this,btShape,width,length,maxHeight,static_cast<uint8_t>(upAxis));
}
util::TSharedHandle<pragma::physics::IGhostObject> pragma::physics::BtEnvironment::CreateGhostObject(IShape &shape)
{
	auto ghost = CreateSharedHandle<BtGhostObject>(*this,std::make_unique<btPairCachingGhostObject>(),shape);
	AddCollisionObject(*ghost);
	return util::shared_handle_cast<BtGhostObject,IGhostObject>(ghost);
}
util::TSharedHandle<pragma::physics::ICollisionObject> pragma::physics::BtEnvironment::CreateCollisionObject(IShape &shape)
{
	auto collisionObject = CreateSharedHandle<pragma::physics::BtCollisionObject>(*this,std::make_unique<btCollisionObject>(),shape);
	AddCollisionObject(*collisionObject);
	return util::shared_handle_cast<BtCollisionObject,ICollisionObject>(collisionObject);
}
util::TSharedHandle<pragma::physics::IRigidBody> pragma::physics::BtEnvironment::CreateRigidBody(IShape &shape,bool dynamic)
{
	auto mass = shape.GetMass();
	if(dynamic == false)
		mass = 0.f;
	auto collisionObject = CreateSharedHandle<BtRigidBody>(*this,dynamic_cast<BtShape&>(shape));
	AddCollisionObject(*collisionObject);
	return util::shared_handle_cast<BtRigidBody,IRigidBody>(collisionObject);
}
static btSoftBody *createSoftBody(btSoftRigidDynamicsWorld *world,btSoftBodyWorldInfo *info,const btScalar s,
	const int numX,
	const int numY, 
	const int fixed) {
	btSoftBody* cloth=btSoftBodyHelpers::CreatePatch(*info,
		btVector3(-s/2,s+1,0),
		btVector3(+s/2,s+1,0),
		btVector3(-s/2,s+1,+s),
		btVector3(+s/2,s+1,+s),
		numX,numY, 
		fixed,true); 
	cloth->getCollisionShape()->setMargin(0.001f);
	cloth->generateBendingConstraints(2,cloth->appendMaterial());
	cloth->setTotalMass(10);  
	cloth->m_cfg.piterations = 5;
	cloth->m_cfg.kDP = 0.005f;
	world->addSoftBody(cloth);
	return cloth;
}
#pragma pack(push,1)
struct Vector3d
{
	Vector3d(double _x,double _y,double _z)
		: x(_x),y(_y),z(_z)
	{}
	Vector3d()=default;
	double x = 0.f;
	double y = 0.f;
	double z = 0.f;
};
#pragma pack(pop)
static std::unique_ptr<btSoftBody> createSoftBody(const PhysSoftBodyInfo &sbInfo,btSoftRigidDynamicsWorld *world,btSoftBodyWorldInfo *info,float mass,std::vector<Vector3d> &verts,std::vector<int32_t> indices) {
	if(mass == 0.f)
	{
		Con::cwar<<"WARNING: Attempted to create soft-body object with mass of 0! Using mass of 1 instead..."<<Con::endl;
		mass = 1.f;
	}
	auto cloth = std::unique_ptr<btSoftBody>{btSoftBodyHelpers::CreateFromTriMesh(*info,reinterpret_cast<btScalar*>(verts.data()),indices.data(),indices.size() /3,true)};

	auto *pm = cloth->appendMaterial();
	auto it = sbInfo.materialStiffnessCoefficient.find(0u);
	if(it != sbInfo.materialStiffnessCoefficient.end())
	{
		pm->m_kLST = it->second.linear;
		pm->m_kAST = it->second.angular;
		pm->m_kVST = it->second.volume;
	}
	pm->m_flags &= ~btSoftBody::fMaterial::DebugDraw;

	cloth->m_cfg.kAHR = sbInfo.anchorsHardness;
	cloth->m_cfg.kCHR = sbInfo.rigidContactsHardness;
	cloth->m_cfg.kDF = sbInfo.dynamicFrictionCoefficient;
	cloth->m_cfg.kDG = sbInfo.dragCoefficient;
	cloth->m_cfg.kDP = sbInfo.dampingCoefficient;
	cloth->m_cfg.kKHR = sbInfo.kineticContactsHardness;
	cloth->m_cfg.kLF = sbInfo.liftCoefficient;
	cloth->m_cfg.kMT = sbInfo.poseMatchingCoefficient;
	cloth->m_cfg.kPR = sbInfo.pressureCoefficient;
	cloth->m_cfg.kSHR = sbInfo.softContactsHardness;
	cloth->m_cfg.kSKHR_CL = sbInfo.softVsKineticHardness;
	cloth->m_cfg.kSK_SPLT_CL = sbInfo.softVsRigidImpulseSplitK;
	cloth->m_cfg.kSRHR_CL = sbInfo.softVsRigidHardness;
	cloth->m_cfg.kSR_SPLT_CL = sbInfo.softVsRigidImpulseSplitR;
	cloth->m_cfg.kSSHR_CL = sbInfo.softVsSoftHardness;
	cloth->m_cfg.kSS_SPLT_CL = sbInfo.softVsRigidImpulseSplitS;
	cloth->m_cfg.kVC = sbInfo.volumeConversationCoefficient;
	cloth->m_cfg.kVCF = sbInfo.velocitiesCorrectionFactor;

	if(sbInfo.bendingConstraintsDistance > 0.f)
		cloth->generateBendingConstraints(sbInfo.bendingConstraintsDistance,pm);

	//cloth->m_cfg.collisions = btSoftBody::fCollision::CL_RS;
	cloth->m_cfg.collisions |= btSoftBody::fCollision::VF_SS;
	// cloth->m_cfg.collisions |= btSoftBody::fCollision::VF_SS | btSoftBody::fCollision::CL_RS | btSoftBody::fCollision::CL_SS;	

	/*
	RVSmask	=	0x000f,	///Rigid versus soft mask
	SDF_RS	=	0x0001,	///SDF based rigid vs soft
	CL_RS	=	0x0002, ///Cluster vs convex rigid vs soft

	SVSmask	=	0x0030,	///Rigid versus soft mask		
	VF_SS	=	0x0010,	///Vertex vs face soft vs soft handling
	CL_SS	=	0x0020, ///Cluster vs cluster soft vs soft handling
	CL_SELF =	0x0040, ///Cluster soft body self collision
	Default	=	SDF_RS,
	END
	*/

	/*cloth->getCollisionShape()->setMargin(0.001f);//0.005f);
	cloth->setTotalMass(mass);  
	cloth->m_cfg.piterations = 5;
	cloth->generateClusters(k,maxiterations);*/
	// cloth->
	//btSoftBody::eAeroModel::F_OneSided

	if(sbInfo.clusterCount > 0)
		cloth->generateClusters(sbInfo.clusterCount,sbInfo.maxClusterIterations);
	cloth->setPose(true,true);

	world->addSoftBody(cloth.get());
	return cloth;
}
util::TSharedHandle<pragma::physics::ISoftBody> pragma::physics::BtEnvironment::CreateSoftBody(const PhysSoftBodyInfo &info,float mass,const std::vector<Vector3> &verts,const std::vector<uint16_t> &indices,std::vector<uint16_t> &indexTranslations)
{
	btAlignedObjectArray<btVector3> vtx;
	vtx.reserve(static_cast<int32_t>(verts.size()));
	for(auto &t : verts)
		vtx.push_back({t.x *pragma::physics::BtEnvironment::WORLD_SCALE,t.y *pragma::physics::BtEnvironment::WORLD_SCALE,t.z *pragma::physics::BtEnvironment::WORLD_SCALE});
#if PHYS_USE_SOFT_RIGID_DYNAMICS_WORLD == 1
	{
		std::vector<Vector3d> sbVerts;
		sbVerts.reserve(vtx.size());
		indexTranslations.resize(vtx.size());
		for(auto i=decltype(vtx.size()){0};i<vtx.size();++i)
		{
			auto &v = vtx.at(i);
			auto it = std::find_if(sbVerts.begin(),sbVerts.end(),[&v](const Vector3d &other) {
				const auto EPSILON = 0.01f;//1.f;
				return umath::abs(other.x -v.x()) /WORLD_SCALE < EPSILON && umath::abs(other.y -v.y()) /WORLD_SCALE < EPSILON && umath::abs(other.z -v.z()) /WORLD_SCALE < EPSILON;
				});
			if(it == sbVerts.end())
			{
				sbVerts.push_back({v.x(),v.y(),v.z()});
				it = sbVerts.end() -1;
			}
			indexTranslations.at(i) = it -sbVerts.begin();
		}
		std::vector<int32_t> sbIndices;
		/*for(auto i=decltype(vtx.size()){0};i<vtx.size();++i)
		{
		auto &v = vtx.at(i);
		sbVerts.push_back({v.x(),v.y(),v.z()});
		//sbVerts.push_back(vtx.at(i).x());
		//sbVerts.push_back(vtx.at(i).y());
		//sbVerts.push_back(vtx.at(i).z());
		}*/
		//	for(auto idx : indices)
		//	sbIndices.push_back(idx);
		//sbIndices.reserve(indices.size());
		for(auto idx : indices)
			sbIndices.push_back(indexTranslations.at(idx));


		/*std::vector<btScalar> vts;
		for(auto i=decltype(vtx.size()){0};i<vtx.size();++i)
		{
		auto &v = vtx.at(i);
		vts.push_back(v.x());
		vts.push_back(v.y());
		vts.push_back(v.z());
		}
		std::vector<int32_t> id;
		for(auto idx : indices)
		{
		id.push_back(idx);
		}*/
		auto body = createSoftBody(info,GetWorld(),GetBtSoftBodyWorldInfo(),mass,sbVerts,sbIndices);
		//auto *body = createSoftBody(GetWorld(),GetBtSoftBodyWorldInfo(),4.f,64,64,1);
		body->activate(true);

		auto *btShape = body->getCollisionShape();
		auto shape = CreateSharedPtr<BtShape>(*this,btShape,false);
		auto softBody = CreateSharedHandle<BtSoftBody>(*this,std::move(body),*shape,indexTranslations);
		AddCollisionObject(*softBody);
		return util::shared_handle_cast<BtSoftBody,ISoftBody>(softBody);
		/*c_game->AddCallback("Tick",FunctionCallback<void>::Create([]() {
		softBody->activate(true);
		//m_dynamicsWorld->stepSimulation(c_game->DeltaTickTime(),1,c_game->DeltaTickTime());
		}));*/
	}
	{
		/*auto s = 256 *pragma::physics::BtEnvironment::WORLD_SCALE;
		auto numX = 64;
		auto numY = 64;
		auto fixed = 1 +2;
		btSoftBody* cloth=btSoftBodyHelpers::CreatePatch(*m_softBodyWorldInfo,
		btVector3(-s/2,s+1,0),
		btVector3(+s/2,s+1,0),
		btVector3(-s/2,s+1,+s),
		btVector3(+s/2,s+1,+s),
		numX,numY, 
		fixed,true); 
		cloth->getCollisionShape()->setMargin(0.001f);
		cloth->generateBendingConstraints(2,cloth->appendMaterial());
		cloth->setTotalMass(10);  
		cloth->m_cfg.piterations = 5;
		cloth->m_cfg.kDP = 0.005f;
		//m_btWorld->addSoftBody(cloth);
		auto shape = std::shared_ptr<PhysShape>(new PhysShape(cloth->getCollisionShape(),false));
		auto *softBody = new PhysSoftBody(this,cloth,shape);
		softBody->Initialize();
		AddSoftBody(softBody);
		return softBody;*/
	}
	/*{
	std::vector<int> triVertexIndices {};
	for(auto idx : indices)
	triVertexIndices.push_back(idx);
	std::vector<float> triVertexCoords {};
	for(auto i=decltype(vtx.size()){0};i<vtx.size();++i)
	{
	auto &v = vtx.at(i);
	triVertexCoords.push_back(v.x());
	triVertexCoords.push_back(v.y());
	triVertexCoords.push_back(v.z());
	}
	auto numTriangles = triVertexIndices.size() / 3;
	auto numVertices = triVertexCoords.size() / 3;
	auto triVertCoords = new btScalar[triVertexCoords.size()];
	auto triVertIndices = new int[triVertexIndices.size()];

	for(int i = 0; i < triVertexCoords.size(); i ++)
	{
	triVertCoords[i] = static_cast<btScalar>(triVertexCoords[i]);


	}


	for(int i = 0; i < triVertexIndices.size(); i ++)
	{
	triVertIndices[i] = triVertexIndices[i];
	}
	//this->m_body.reset( );
	auto *body = btSoftBodyHelpers::CreateFromTriMesh(*m_softBodyWorldInfo, triVertCoords, triVertIndices, numTriangles);

	btSoftBody::Material*   pm=body->appendMaterial();
	pm->m_kLST            =   0.5;
	pm->m_flags            -=   btSoftBody::fMaterial::DebugDraw;
	body->generateBendingConstraints(2,pm);
	body->m_cfg.piterations   =   3;
	body->m_cfg.kDF         =   0.5;
	body->m_cfg.collisions |= btSoftBody::fCollision::VF_SS;	
	body->scale(btVector3(1,1,1));   
	body->setTotalMass(100, true);
	body->randomizeConstraints();
	body->getCollisionShape()->setMargin(0.1f);

	auto *btShape = body->getCollisionShape();
	auto shape = std::shared_ptr<PhysShape>(new PhysShape(btShape,false));
	auto *softBody = new PhysSoftBody(this,body,shape);
	softBody->Initialize();
	AddSoftBody(softBody);
	return softBody;
	}
	auto *psb = new btSoftBody(m_softBodyWorldInfo.get(),vtx.size(),&vtx[0],0);
	uint32_t i = 0;
	uint32_t ni = 0;
	auto ntriangles = indices.size() /3;
	auto &triangles = indices;
	auto maxidx = vtx.size();
	btAlignedObjectArray<bool>		chks;
	chks.resize(maxidx*maxidx,false);
	for( i=0,ni=ntriangles*3;i<ni;i+=3)
	{
	const int idx[]={triangles[i],triangles[i+1],triangles[i+2]};
	#define IDX(_x_,_y_) ((_y_)*maxidx+(_x_))
	for(int j=2,k=0;k<3;j=k++)
	{
	if(!chks[IDX(idx[j],idx[k])])
	{
	chks[IDX(idx[j],idx[k])]=true;
	chks[IDX(idx[k],idx[j])]=true;
	psb->appendLink(idx[j],idx[k]);
	}
	}
	#undef IDX
	psb->appendFace(idx[0],idx[1],idx[2]);
	}*/
	/*std::vector<int32_t> ind {};
	ind.reserve(indices.size());
	for(auto idx : indices)
	ind.push_back(idx);
	auto *psb = btSoftBodyHelpers::CreateFromTriMesh(*m_softBodyWorldInfo,reinterpret_cast<btScalar*>(&vtx[0]),ind.data(),ind.size() /3);*/
	/*this->numTriangles = triVertexIndices.size() / 3;
	this->numVertices = triVertexCoords.size() / 3;
	this->triVertCoords = new btScalar[triVertexCoords.size()];
	this->triVertIndices = new int[triVertexIndices.size()];

	for(int i = 0; i < triVertexCoords.size(); i ++)
	{
	this->triVertCoords[i] = static_cast<btScalar>(triVertexCoords[i]);


	}


	for(int i = 0; i < triVertexIndices.size(); i ++)
	{
	this->triVertIndices[i] = triVertexIndices[i];
	}
	this->m_body.reset( btSoftBodyHelpers::CreateFromTriMesh(worldInfo, this->triVertCoords, this->triVertIndices, this->numTriangles));*/

	/*auto *pm = psb->appendMaterial(); // TODO
	pm->m_kLST = 0.55;
	pm->m_kAST = 0.2;
	pm->m_kVST = 0.55;*/
	/*for(auto i=decltype(indices.size()){0};i<indices.size();i+=3)
	{
	psb->appendLink(indices[i +2],indices[i],pm,true);
	psb->appendLink(indices[i],indices[i +1],pm,true);
	psb->appendLink(indices[i +1],indices[i +2],pm,true);

	psb->appendFace(indices[i],indices[i +1],indices[i +2],pm);
	}*/

	/*psb->generateBendingConstraints(2,pm);
	psb->m_cfg.piterations = 2;
	psb->m_cfg.collisions |= btSoftBody::fCollision::VF_SS;// | btSoftBody::fCollision::CL_SS;

	psb->generateBendingConstraints(2,pm);
	psb->m_cfg.piterations = 3;
	psb->m_cfg.kDF = 0.5;
	psb->scale(btVector3(1,1,1));   
	psb->setTotalMass(100,true);
	//	psb->randomizeConstraints();

	btTransform trans;
	trans.setIdentity();
	psb->setWorldTransform(trans);
	//	psb->setTotalMass(1.0,true);
	//	psb->generateClusters(64);

	auto *btShape = psb->getCollisionShape();
	auto shape = std::shared_ptr<PhysShape>(new PhysShape(btShape,false));
	auto *softBody = new PhysSoftBody(this,psb,shape);
	softBody->Initialize();
	AddSoftBody(softBody);
	return softBody;*/
#endif
}
void pragma::physics::BtEnvironment::AddAction(btActionInterface *action) {m_btWorld->addAction(action);}

namespace pragma::physics
{
	template<class TBase>
		class PhysRayResultCallback
		: public TBase
	{
	public:
		PhysRayResultCallback(pragma::physics::IRayCastFilterCallback *filter,const btVector3 &rayFromWorld,const btVector3 &rayToWorld)
			: m_filter{filter},TBase{rayFromWorld,rayToWorld}
		{}
		virtual btScalar addSingleResult(btCollisionWorld::LocalRayResult &rayResult,bool normalInWorldSpace) override
		{
			auto *obj = rayResult.m_collisionObject;
			auto *colObj = static_cast<pragma::physics::ICollisionObject*>(obj->getUserPointer());
			if(colObj != nullptr)
			{
				auto *physObj = colObj->GetPhysObj();
				if(physObj != nullptr)
				{
					auto *ent = physObj->GetOwner();
					auto *pPhysComponent = (ent != nullptr) ? ent->GetEntity().GetPhysicsComponent() : nullptr;
					if(pPhysComponent && pPhysComponent->IsRayResultCallbackEnabled() == true)
					{
						if(pPhysComponent->RayResultCallback(
							static_cast<CollisionMask>(m_collisionFilterGroup),static_cast<CollisionMask>(m_collisionFilterMask)
						) == false)
							return 0.0;
					}
					// TODO: Filter
				}
			}
			return TBase::addSingleResult(rayResult,normalInWorldSpace);
		}
	private:
		pragma::physics::IRayCastFilterCallback *m_filter = nullptr;
	};

	class PhysAllHitsRayResultCallback
		: public PhysRayResultCallback<btCollisionWorld::AllHitsRayResultCallback>
	{
	public:
		using PhysRayResultCallback<btCollisionWorld::AllHitsRayResultCallback>::PhysRayResultCallback;
	};

	class PhysClosestRayResultCallback
		: public PhysRayResultCallback<btCollisionWorld::ClosestRayResultCallback>
	{
	public:
		using PhysRayResultCallback<btCollisionWorld::ClosestRayResultCallback>::PhysRayResultCallback;
	};
};

Bool pragma::physics::BtEnvironment::RayCast(const TraceData &data,std::vector<TraceResult> *results) const
{
	auto origin = data.GetSourceOrigin();
	auto btOrigin = uvec::create_bt(origin) *WORLD_SCALE;
	auto btEnd = uvec::create_bt(data.GetTargetOrigin()) *WORLD_SCALE;
#ifdef _DEBUG
	//if(uvec::cmp(origin,data.GetTargetOrigin()) == true)
	//	btEnd.setX(btEnd.getX() +1.f); // Move it slightly, so source and target aren't the same (Causes fuzzyZero assertion error in debug mode)
#endif
	auto &filter = data.GetFilter();
	auto flags = data.GetFlags();
	auto mask = data.GetCollisionFilterMask();
	auto group = data.GetCollisionFilterGroup();
	if((UInt32(flags) &UInt32(RayCastFlags::ReportAllResults)) == 0 || results == nullptr)
	{
		pragma::physics::PhysClosestRayResultCallback btResult {filter.get(),btOrigin,btEnd};
		m_btWorld->rayTest(btOrigin,btEnd,btResult);
		if(results != nullptr)
		{
			results->push_back(TraceResult());
			auto &r = results->back();
			r.startPosition = origin;
			if(btResult.hasHit())
				r.hitType = RayCastHitType::Block;
			else
				r.hitType = RayCastHitType::None;

			r.fraction = CFloat(btResult.m_closestHitFraction);
			if(r.hitType == RayCastHitType::None)
				r.position = data.GetTargetOrigin();
			else
				r.position = uvec::create(btResult.m_hitPointWorld /WORLD_SCALE);
			r.distance = uvec::distance(r.position,origin);
			r.normal = uvec::create(btResult.m_hitNormalWorld);
		}
		if(btResult.hasHit() == true)
		{
			if(results == nullptr)
				return true;
			auto *obj = btResult.m_collisionObject;
			auto *colObj = static_cast<ICollisionObject*>(obj->getUserPointer());
			if(colObj != nullptr)
			{
				auto &r = results->back();
				r.collisionObj = util::weak_shared_handle_cast<IBase,ICollisionObject>(colObj->GetHandle());
				auto *physObj = colObj->GetPhysObj();
				if(physObj != nullptr)
				{
					r.physObj = physObj->GetHandle();
					auto *ent = physObj->GetOwner();
					if(ent != nullptr)
						r.entity = &ent->GetEntity();
				}
			}
		}
		return btResult.hasHit();
	}
	else
	{
		pragma::physics::PhysAllHitsRayResultCallback btResult {filter.get(),btOrigin,btEnd};
		m_btWorld->rayTest(btOrigin,btEnd,btResult);
		if(btResult.hasHit() == true)
		{
			for(int i=0;i<btResult.m_collisionObjects.size();i++)
			{
				results->push_back(TraceResult());
				auto &r = results->back();
				r.startPosition = origin;
				r.hitType = RayCastHitType::Block;
				r.position = uvec::create(btResult.m_hitPointWorld[i] /WORLD_SCALE);
				r.distance = uvec::distance(r.position,origin);
				r.normal = uvec::create(btResult.m_hitNormalWorld[i]);
				if(i == 0)
					r.fraction = CFloat(btResult.m_closestHitFraction);
				else
				{
					auto distance = data.GetDistance();
					r.fraction = (distance != 0.f) ? r.distance /distance : 0.f;
				}
				auto *obj = btResult.m_collisionObjects[i];
				auto *colObj = static_cast<ICollisionObject*>(obj->getUserPointer());
				if(colObj != nullptr)
				{
					r.collisionObj = util::weak_shared_handle_cast<IBase,ICollisionObject>(colObj->GetHandle());
					auto *physObj = colObj->GetPhysObj();
					if(physObj != nullptr)
					{
						r.physObj = physObj->GetHandle();
						auto *ent = physObj->GetOwner();
						if(ent != nullptr)
							r.entity = &ent->GetEntity();
					}
				}
			}
		}
		return btResult.hasHit();
	}
	return false;
}

Bool pragma::physics::BtEnvironment::Overlap(const TraceData &data,std::vector<TraceResult> *results) const
{
#if 0
	auto *shape = data.GetShape();
	if(shape == nullptr)
		return false;
	auto &filter = data.GetFilter();
	auto flags = data.GetFlags();
	auto group = data.GetCollisionFilterGroup();
	auto mask = data.GetCollisionFilterMask();
	auto btResult = (filter != nullptr) ? filter->CreateContactCallbackFilter(flags,group,mask) : PhysContactResultCallback(flags,group,mask);
	for(auto it=objs.begin();it!=objs.end();it++)
	{
		auto *o = *it;
		auto *btObj = o->GetCollisionObject();
		auto btTransform = btObj->getWorldTransform();
		if(data.HasTarget())
		{
			auto &t = data.GetTarget();
			btTransform = t.GetTransform();
		}
		btVector3 aabbMin,aabbMax;
		btObj->getCollisionShape()->getAabb(btTransform,aabbMin,aabbMax);
		btSingleContactCallbackCustom contactCB(btObj,m_btWorld.get(),btResult);
		auto broadPhase = m_btWorld->getBroadphase();
		broadPhase->aabbTest(aabbMin,aabbMax,contactCB);
		/*m_btWorld->contactTest(btObj,btResult);*/
		if(btResult.hasHit == true)
			break;
	}
	if(result == nullptr)
		return btResult.hasHit;
	auto &r = *result;
	r.hit = btResult.hasHit;
	r.position = btResult.m_positionWorldOnA;
	auto origin = data.GetSourceOrigin();
	r.distance = uvec::distance(r.position,origin);
	if(btResult.hasHit == true)
	{
		auto *obj = btResult.m_colObj;
		auto *colObj = static_cast<ICollisionObject*>(obj->getUserPointer());
		if(colObj != nullptr)
		{
			r.collisionObj = colObj->GetHandle();
			auto *physObj = colObj->GetPhysObj();
			if(physObj != nullptr)
			{
				r.physObj = physObj->GetHandle();
				auto *ent = physObj->GetOwner();
				if(ent != nullptr)
					r.entity = &ent->GetEntity();
			}
		}
	}
	return btResult.hasHit;
#endif
	return false;
}
Bool pragma::physics::BtEnvironment::Sweep(const TraceData &data,std::vector<TraceResult> *results) const
{
#if 0
	std::vector<btConvexShape*> shapes;
	if(data.GetShapes(shapes) == false)
		return false;
	auto *filter = data.GetFilter();
	auto flags = data.GetFlags();
	auto group = data.GetCollisionFilterGroup();
	auto mask = data.GetCollisionFilterMask();
	auto btOrigin = uvec::create_bt(data.GetSourceOrigin()) *CFloat(WORLD_SCALE);
	auto btEnd = uvec::create_bt(data.GetTargetOrigin()) *CFloat(WORLD_SCALE);
	auto &tStart = data.GetSource();
	auto &tEnd = data.GetTarget();
	auto btResult = (filter != nullptr) ? filter->CreateClosestConvexCallbackFilter(flags,group,mask,btOrigin,btEnd) : PhysClosestConvexResultCallback(btOrigin,btEnd,flags,group,mask);
	for(auto it=shapes.begin();it!=shapes.end();it++)
	{
		auto *shape = *it;
		m_btWorld->convexSweepTest(shape,tStart.GetTransform(),tEnd.GetTransform(),btResult);
		if(btResult.hasHit() == true)
			break;
	}
	if(result != nullptr)
	{
		auto &r = *result;
		r.startPosition = data.GetSourceOrigin();
		r.hit = btResult.hasHit();
		r.fraction = CFloat(btResult.m_closestHitFraction);
		if(r.hit == false)
			r.position = data.GetTargetOrigin();
		else
			r.position = uvec::create(btResult.m_hitPointWorld /WORLD_SCALE);
		r.distance = uvec::distance(r.position,tStart.GetOrigin());
		r.normal = uvec::create(btResult.m_hitNormalWorld);
		if(btResult.hasHit() == true)
		{
			auto *obj = btResult.m_hitCollisionObject;
			auto *colObj = static_cast<PhysCollisionObject*>(obj->getUserPointer());
			if(colObj != nullptr)
			{
				r.collisionObj = colObj->GetHandle();
				auto *physObj = colObj->GetPhysObj();
				if(physObj != nullptr)
				{
					r.physObj = physObj->GetHandle();
					auto *ent = physObj->GetOwner();
					if(ent != nullptr)
						r.entity = &ent->GetEntity();
				}
			}
		}
	}
	return btResult.hasHit();
#endif
	return false;
}
void pragma::physics::BtEnvironment::RemoveConstraint(IConstraint &constraint)
{
	m_btWorld->removeConstraint(&dynamic_cast<BtConstraint&>(constraint).GetInternalObject());
	IEnvironment::RemoveConstraint(constraint);

}
void pragma::physics::BtEnvironment::RemoveCollisionObject(ICollisionObject &obj)
{
	m_contactMap.RemoveContacts(dynamic_cast<BtCollisionObject&>(obj).GetBtCollisionObject());
	m_btWorld->removeCollisionObject(&dynamic_cast<BtCollisionObject&>(obj).GetInternalObject());
	IEnvironment::RemoveCollisionObject(obj);

}
void pragma::physics::BtEnvironment::RemoveController(IController &controller)
{
	m_btWorld->removeAction(dynamic_cast<BtController&>(controller).GetCharacterController());
	IEnvironment::RemoveController(controller);

}
pragma::physics::IEnvironment::RemainingDeltaTime pragma::physics::BtEnvironment::DoStepSimulation(float timeStep,int maxSubSteps,float fixedTimeStep)
{
	if(fixedTimeStep == 0.f)
		return timeStep;
	for(auto &hController : GetControllers())
	{
		if(hController.IsExpired())
			continue;
		auto *controller = dynamic_cast<BtController*>(hController.Get());
		if(controller == nullptr)
			continue;
		controller->PreSimulate(timeStep);
	}
	g_simEnvironment = this;
	m_btWorld->stepSimulation(timeStep,maxSubSteps,fixedTimeStep);
	for(auto &hController : GetControllers())
	{
		if(hController.IsExpired())
			continue;
		auto *controller = dynamic_cast<BtController*>(hController.Get());
		if(controller == nullptr)
			continue;
		controller->PostSimulate(timeStep);
	}
	if(m_btDebugDrawer)
	{
		auto *pVisDebugger = GetVisualDebugger();
		pVisDebugger->Reset();
		m_btWorld->debugDrawWorld();
		pVisDebugger->Flush();
	}
	g_simEnvironment = nullptr;
	return fmodf(timeStep,fixedTimeStep);
}

// Update physics info for character controllers
static void update_physics_contact_controller_info(Game *game,int idx,const btCollisionObject *o,const btCollisionObject *oOther,btPersistentManifold *contactManifold)
{
	auto *col = static_cast<pragma::physics::ICollisionObject*>(o->getUserPointer());
	if(col == nullptr)
		return;
	auto *phys = col->GetPhysObj();
	auto *colOther = static_cast<pragma::physics::ICollisionObject*>(oOther->getUserPointer());
	if(phys == nullptr || phys->IsController() == false || (colOther != nullptr && colOther->IsTrigger() == true))
		return;
	auto *shape = o->getCollisionShape();
	auto numContacts = contactManifold->getNumContacts();
	for(auto i=decltype(numContacts){0};i<numContacts;++i)
	{
		auto &contactPoint = contactManifold->getContactPoint(i);
		auto &localPoint = (idx == 0) ? contactPoint.m_localPointA : contactPoint.m_localPointB;
		auto y = localPoint.y();

		// If the contact point is straight below the controller, assume this means the controller is standing on solid ground!
		const auto BOX_SHAPE_MARGIN_EPSILON = 2.0 *pragma::physics::BtEnvironment::WORLD_SCALE;
		auto bValidCandidate = false;
		switch(shape->getShapeType())
		{
		case CAPSULE_SHAPE_PROXYTYPE: // Check if contact point is touching lower half-sphere of capsule shape
			bValidCandidate = y < static_cast<const btCapsuleShape*>(shape)->getHalfHeight();
			break;
		case BOX_SHAPE_PROXYTYPE: // Check if contact point is touching bottom of box shape
			bValidCandidate = y < static_cast<const btBoxShape*>(shape)->getHalfExtentsWithoutMargin().y() +BOX_SHAPE_MARGIN_EPSILON;
		}
		if(bValidCandidate == false)
			continue;
		// This will only be applied if the contact point is a better candidate than the controller's previous candidate (for this tick)!
		auto *controller = static_cast<ControllerPhysObj*>(phys)->GetController();
		if(controller)
			dynamic_cast<pragma::physics::BtController*>(controller)->SetGroundContactPoint(contactPoint,idx,o,oOther);
	}
}

void pragma::physics::BtEnvironment::SimulationCallback(double)
{
	// auto t = std::chrono::high_resolution_clock::now();
	btDispatcher *dispatcher = m_btWorld->getDispatcher();
	int numManifolds = dispatcher->getNumManifolds();
	auto &nw = GetNetworkState();
	auto *game = nw.GetGameState();
	auto bClient = game->IsClient();
	CollisionContactList newContacts {};
	for(int i=0;i<numManifolds;i++)
	{
		btPersistentManifold *contactManifold =  dispatcher->getManifoldByIndexInternal(i);
		int numContacts = contactManifold->getNumContacts();
		if(numContacts > 0)
		{
			// Update character ground contact point
			auto *o0 = contactManifold->getBody0();
			auto *o1 = contactManifold->getBody1();
			update_physics_contact_controller_info(game,0,o0,o1,contactManifold);
			update_physics_contact_controller_info(game,1,o1,o0,contactManifold);
			//

			// Physics Sound
			if(bClient == true)
			{
				// TODO: This doesn't belong here! Move to Engine!
				auto *obj = static_cast<pragma::physics::ICollisionObject*>(o1->getUserPointer());
				if(obj != nullptr)
				{
					auto *surface = game->GetSurfaceMaterial(obj->GetSurfaceMaterial());
					if(surface != nullptr)
					{
						for(auto i=0;i<numContacts;i++)
						{
							auto &pt = contactManifold->getContactPoint(0);
							if(pt.getDistance() <= 1.f *pragma::physics::BtEnvironment::WORLD_SCALE)
							{
								auto impulse = pt.getAppliedImpulse(); // Musn't be scaled by world scale!
								const auto softImpactThreshold = 100.f;
								const auto hardImpactThreshold = 400.f;
								if(impulse >= softImpactThreshold)
								{
									auto bHardImpact = (impulse >= hardImpactThreshold) ? true : false;
									std::string sndImpact = (bHardImpact == false) ? surface->GetSoftImpactSound() : surface->GetHardImpactSound();
									if(sndImpact.empty())
									{
										surface = game->GetSurfaceMaterial(0);
										sndImpact = (bHardImpact == false) ? surface->GetSoftImpactSound() : surface->GetHardImpactSound();
									}
									auto snd = nw.CreateSound(sndImpact,ALSoundType::Effect | ALSoundType::Physics,ALCreateFlags::Mono);
									if(snd != nullptr)
									{
										auto pos = pt.getPositionWorldOnB() /pragma::physics::BtEnvironment::WORLD_SCALE;
										snd->SetPosition(Vector3(pos.x(),pos.y(),pos.z()));
										if(impulse < hardImpactThreshold)
											snd->SetGain(CFloat(0.25f +(impulse -softImpactThreshold) /(hardImpactThreshold -softImpactThreshold) *0.5f));
										else
											snd->SetGain(CFloat(0.75f +(impulse -hardImpactThreshold) /hardImpactThreshold *0.25f));
										snd->Play();
									}
									break;
								}
							}
						}
					}
				}
			}
			//

			newContacts.insert(std::make_pair(o0,o1));
			newContacts.insert(std::make_pair(o1,o0));
			m_contactMap.AddContact(*o0,*o1);
			/*auto *colA = static_cast<pragma::physics::ICollisionObject*>(o0->getUserPointer());
			PhysObj *physA = (colA != nullptr) ? colA->GetPhysObj() : nullptr;
			auto *entA = (physA != nullptr) ? &physA->GetOwner()->GetEntity() : nullptr;
			auto pPhysComponentA = (entA != nullptr) ? entA->GetPhysicsComponent() : nullptr;

			auto *colB = static_cast<pragma::physics::ICollisionObject*>(o1->getUserPointer());
			PhysObj *physB = (colB != nullptr) ? colB->GetPhysObj() : nullptr;
			auto *entB = (physB != nullptr) ? &physB->GetOwner()->GetEntity() : nullptr;
			auto pPhysComponentB = (entB != nullptr) ? entB->GetPhysicsComponent() : nullptr;
			if(pPhysComponentA && pPhysComponentB)*/
			{
#if 0
				bool bCallbackA = pPhysComponentA->GetCollisionCallbacksEnabled();
				bool bCallbackB = pPhysComponentB->GetCollisionCallbacksEnabled();





				if(bCallbackA == true || bCallbackB == true)
				{
					auto *touchComponentA = static_cast<pragma::BaseTouchComponent*>(entA->FindComponent("touch").get());
					auto *touchComponentB = static_cast<pragma::BaseTouchComponent*>(entB->FindComponent("touch").get());
					if(bCallbackA == true && touchComponentA != nullptr)
						touchComponentA->Touch(entB,physB,colA,colB);
					if(bCallbackB == true && touchComponentB != nullptr)
						touchComponentB->Touch(entA,physA,colB,colA);
					bool bReportA = pPhysComponentA->GetCollisionContactReportEnabled();
					bool bReportB = pPhysComponentB->GetCollisionContactReportEnabled();
					if(bReportA == true || bReportB == true)
					{
						for(int j=0;j<numContacts;j++)
						{
							btManifoldPoint &pt = contactManifold->getContactPoint(j);
							if(pt.getDistance() < 0.f)
							{
								const btVector3 &ptA = pt.getPositionWorldOnA();
								const btVector3 &ptB = pt.getPositionWorldOnB();
								const btVector3 &normalOnB = pt.m_normalWorldOnB;

								if(bReportA == true && touchComponentA != nullptr)
								{
									PhysContact contact {};
									contact.entA = entA;
									contact.entB = entB;
									contact.physA = physA;
									contact.physB = physB;
									contact.objA = colA;
									contact.objB = colB;
									contact.posA = Vector3(ptA.x(),ptA.y(),ptA.z()) /static_cast<float>(pragma::physics::BtEnvironment::WORLD_SCALE);
									contact.posB = Vector3(ptB.x(),ptB.y(),ptB.z()) /static_cast<float>(pragma::physics::BtEnvironment::WORLD_SCALE);
									contact.hitNormal = Vector3(normalOnB.x(),normalOnB.y(),normalOnB.z());
									touchComponentA->Contact(contact);
								}
								if(bReportB == true && touchComponentB != nullptr)
								{
									PhysContact contact {};
									contact.entA = entB;
									contact.entB = entA;
									contact.physA = physB;
									contact.physB = physA;
									contact.objA = colB;
									contact.objB = colA;
									contact.posA = Vector3(ptB.x(),ptB.y(),ptB.z()) /static_cast<float>(pragma::physics::BtEnvironment::WORLD_SCALE);
									contact.posB = Vector3(ptA.x(),ptA.y(),ptA.z()) /static_cast<float>(pragma::physics::BtEnvironment::WORLD_SCALE);
									contact.hitNormal = Vector3(normalOnB.x(),normalOnB.y(),normalOnB.z());
									touchComponentB->Contact(contact);
								}
							}
						}
					}
				}
#endif
			}
		}
	}

	m_contactMap.UpdateContacts(newContacts);
}

util::TSharedHandle<pragma::physics::IController> pragma::physics::BtEnvironment::CreateCapsuleController(
	float halfWidth,float halfHeight,float stepHeight,float slopeLimitDeg,const umath::Transform &startTransform
)
{
	auto shape = CreateCapsuleShape(halfWidth,halfHeight,GetGenericMaterial());
	auto ghostObject = util::shared_handle_cast<IGhostObject,BtGhostObject>(CreateGhostObject(*shape));
	ghostObject->SetWorldTransform(startTransform);

	ghostObject->SetCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT);
	ghostObject->SetContactProcessingThreshold(btScalar(0.0));

	auto &btGhostObject = ghostObject->GetInternalObject();
	auto &btShape = dynamic_cast<BtConvexShape&>(*shape).GetBtConvexShape();

	auto controller = std::unique_ptr<PhysKinematicCharacterController>(new PhysKinematicCharacterController{&btGhostObject,&btShape,btScalar(stepHeight *WORLD_SCALE),btVector3{0.0,1.0,0.0}});
	controller->setGravity({0.0,0.0,0.0});
	controller->setUseGhostSweepTest(false); // If set to true => causes penetration issues with convex meshes, resulting in bouncy physics
	controller->setMaxSlope(umath::deg_to_rad(slopeLimitDeg));
	AddAction(controller.get());
	Vector3 halfExtents {halfWidth,halfHeight,halfWidth};
	auto btController =  util::shared_handle_cast<BtController,IController>(CreateSharedHandle<pragma::physics::BtController>(*this,std::dynamic_pointer_cast<BtConvexShape>(shape),util::shared_handle_cast<BtGhostObject,IGhostObject>(ghostObject),std::move(controller),halfExtents,IController::ShapeType::Capsule));
	AddController(*btController);
	return btController;
}

util::TSharedHandle<pragma::physics::IController> pragma::physics::BtEnvironment::CreateBoxController(
	const Vector3 &halfExtents,float stepHeight,float slopeLimitDeg,const umath::Transform &startTransform
)
{
	auto shape = CreateBoxShape(halfExtents,GetGenericMaterial());
	auto ghostObject = util::shared_handle_cast<IGhostObject,BtGhostObject>(CreateGhostObject(*shape));
	ghostObject->SetWorldTransform(startTransform);

	ghostObject->SetCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT);
	ghostObject->SetContactProcessingThreshold(btScalar(0.0));

	auto &btGhostObject = ghostObject->GetInternalObject();
	auto &btShape = dynamic_cast<BtConvexShape&>(*shape).GetBtConvexShape();

	auto controller = std::unique_ptr<PhysKinematicCharacterController>(new PhysKinematicCharacterController{&btGhostObject,&btShape,btScalar(stepHeight *WORLD_SCALE)});
	controller->setGravity({0.0,0.0,0.0});
	controller->setUseGhostSweepTest(false); // If set to true => causes penetration issues with convex meshes, resulting in bouncy physics
	controller->setMaxSlope(umath::deg_to_rad(slopeLimitDeg));

	auto *world = GetWorld();
	AddAction(controller.get());
	return util::shared_handle_cast<BtController,IController>(CreateSharedHandle<pragma::physics::BtController>(*this,std::dynamic_pointer_cast<BtConvexShape>(shape),util::shared_handle_cast<BtGhostObject,IGhostObject>(ghostObject),std::move(controller),halfExtents,IController::ShapeType::Capsule));
}

////////////////

void pragma::physics::ContactMap::UpdateContacts(const CollisionContactList &newContacts)
{
	for(auto it=m_contacts.begin();it!=m_contacts.end();)
	{
		auto it2 = newContacts.find(*it);
		if(it2 == newContacts.end())
		{
			it = ClearContact(it);
			continue;
		}
		++it;
	}
}
void pragma::physics::ContactMap::AddContact(const btCollisionObject &a,const btCollisionObject &b)
{
	auto pair = std::make_pair(&a,&b);
	auto it = m_contacts.find(pair);
	if(it != m_contacts.end())
		return;
	m_contacts.insert(pair);
	auto *ao = static_cast<pragma::physics::ICollisionObject*>(a.getUserPointer());
	auto *bo = static_cast<pragma::physics::ICollisionObject*>(b.getUserPointer());
	if(ao && bo)
		ao->OnStartTouch(*bo);
	AddContact(b,a);
}
void pragma::physics::ContactMap::RemoveContacts(const btCollisionObject &o)
{
	// TODO: This isn't very efficient...
	for(auto it=m_contacts.begin();it!=m_contacts.end();)
	{
		if(it->first == &o || it->second == &o)
			it = ClearContact(it);
		else
			++it;
	}
}
pragma::physics::CollisionContactList::iterator pragma::physics::ContactMap::ClearContact(CollisionContactList::iterator it)
{
	auto *ao = static_cast<pragma::physics::ICollisionObject*>(it->first->getUserPointer());
	auto *bo = static_cast<pragma::physics::ICollisionObject*>(it->second->getUserPointer());
	if(ao && bo)
		ao->OnEndTouch(*bo);
	return m_contacts.erase(it);
}
void pragma::physics::ContactMap::RemoveContact(const btCollisionObject &a,const btCollisionObject &b)
{
	auto it = m_contacts.find(std::make_pair(&a,&b));
	if(it != m_contacts.end())
		it = ClearContact(it);
	it = m_contacts.find(std::make_pair(&b,&a));
	if(it != m_contacts.end())
		it = ClearContact(it);
}
pragma::physics::ContactMap::~ContactMap()
{
	for(auto it=m_contacts.begin();it!=m_contacts.end();)
		it = ClearContact(it);
}
