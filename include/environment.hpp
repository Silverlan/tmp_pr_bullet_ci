#ifndef __PR_BT_ENVIRONMENT_HPP__
#define __PR_BT_ENVIRONMENT_HPP__

#include "common.hpp"
#include <pragma/physics/environment.hpp>
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
		static Transform CreateTransform(const btTransform &btTransform);
		static btTransform CreateBtTransform(const Transform &btTransform);
		BtEnvironment(NetworkState &state);
		virtual ~BtEnvironment() override;

		BtRigidBody &ToBtType(IRigidBody &body);

		virtual float GetWorldScale() const override;
		virtual IVisualDebugger *InitializeVisualDebugger() override;

		virtual util::TSharedHandle<IFixedConstraint> CreateFixedConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB) override;
		virtual util::TSharedHandle<IBallSocketConstraint> CreateBallSocketConstraint(IRigidBody &a,const Vector3 &pivotA,IRigidBody &b,const Vector3 &pivotB) override;
		virtual util::TSharedHandle<IHingeConstraint> CreateHingeConstraint(IRigidBody &a,const Vector3 &pivotA,IRigidBody &b,const Vector3 &pivotB,const Vector3 &axis) override;
		virtual util::TSharedHandle<ISliderConstraint> CreateSliderConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB) override;
		virtual util::TSharedHandle<IConeTwistConstraint> CreateConeTwistConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB) override;
		virtual util::TSharedHandle<IDoFConstraint> CreateDoFConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB) override;
		virtual util::TSharedHandle<IDoFSpringConstraint> CreateDoFSpringConstraint(IRigidBody &a,const Vector3 &pivotA,const Quat &rotA,IRigidBody &b,const Vector3 &pivotB,const Quat &rotB) override;

		virtual util::TSharedHandle<IController> CreateCapsuleController(float halfWidth,float halfHeight,float stepHeight,float slopeLimitDeg=DEFAULT_CHARACTER_SLOPE_LIMIT,const Transform &startTransform={}) override;
		virtual util::TSharedHandle<IController> CreateBoxController(const Vector3 &halfExtents,float stepHeight,float slopeLimitDeg=DEFAULT_CHARACTER_SLOPE_LIMIT,const Transform &startTransform={}) override;
		virtual util::TSharedHandle<ICollisionObject> CreateCollisionObject(IShape &shape) override;
		virtual util::TSharedHandle<IRigidBody> CreateRigidBody(float mass,IShape &shape,const Vector3 &localInertia) override;
		virtual util::TSharedHandle<ISoftBody> CreateSoftBody(const PhysSoftBodyInfo &info,float mass,const std::vector<Vector3> &verts,const std::vector<uint16_t> &indices,std::vector<uint16_t> &indexTranslations) override;
		virtual util::TSharedHandle<IGhostObject> CreateGhostObject(IShape &shape) override;

		virtual std::shared_ptr<IConvexShape> CreateCapsuleShape(float halfWidth,float halfHeight,const IMaterial &mat) override;
		virtual std::shared_ptr<IConvexShape> CreateBoxShape(const Vector3 &halfExtents,const IMaterial &mat) override;
		virtual std::shared_ptr<IConvexShape> CreateCylinderShape(float radius,float height,const IMaterial &mat) override;
		virtual std::shared_ptr<ICompoundShape> CreateTorusShape(uint32_t subdivisions,double outerRadius,double innerRadius,const IMaterial &mat) override;
		virtual std::shared_ptr<IConvexShape> CreateSphereShape(float radius,const IMaterial &mat) override;
		virtual std::shared_ptr<IConvexHullShape> CreateConvexHullShape(const IMaterial &mat) override;
		virtual std::shared_ptr<ITriangleShape> CreateTriangleShape(const IMaterial &mat) override;
		virtual std::shared_ptr<ICompoundShape> CreateCompoundShape() override;
		virtual std::shared_ptr<ICompoundShape> CreateCompoundShape(IShape &shape) override;
		virtual std::shared_ptr<ICompoundShape> CreateCompoundShape(std::vector<IShape*> &shapes) override;
		virtual std::shared_ptr<IShape> CreateHeightfieldTerrainShape(uint32_t width,uint32_t length,Scalar maxHeight,uint32_t upAxis,const IMaterial &mat) override;
		virtual std::shared_ptr<IMaterial> CreateMaterial(float staticFriction,float dynamicFriction,float restitution) override;

		virtual RemainingDeltaTime StepSimulation(float timeStep,int maxSubSteps=1,float fixedTimeStep=(1.f /60.f)) override;

		virtual Bool Overlap(const TraceData &data,std::vector<TraceResult> *results=nullptr) const override;
		virtual Bool RayCast(const TraceData &data,std::vector<TraceResult> *results=nullptr) const override;
		virtual Bool Sweep(const TraceData &data,TraceResult *result=nullptr) const override;

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

		// For internal or debugging purposes only!
		util::TSharedHandle<IFixedConstraint> AddFixedConstraint(std::unique_ptr<btFixedConstraint> c);
		util::TSharedHandle<IBallSocketConstraint> AddBallSocketConstraint(std::unique_ptr<btPoint2PointConstraint> c);
		util::TSharedHandle<IHingeConstraint> AddHingeConstraint(std::unique_ptr<btHingeConstraint> c);
		util::TSharedHandle<ISliderConstraint> AddSliderConstraint(std::unique_ptr<btSliderConstraint> c);
		util::TSharedHandle<IConeTwistConstraint> AddConeTwistConstraint(std::unique_ptr<btConeTwistConstraint> c);
		util::TSharedHandle<IDoFConstraint> AddDoFConstraint(std::unique_ptr<btGeneric6DofConstraint> c);
		util::TSharedHandle<IDoFSpringConstraint> AddDoFSpringConstraint(std::unique_ptr<btGeneric6DofSpring2Constraint> c);
	protected:
		std::unique_ptr<btWorldType> m_btWorld = nullptr;
		std::unique_ptr<btDefaultCollisionConfiguration> m_btCollisionConfiguration = nullptr;
		std::unique_ptr<btCollisionDispatcher> m_btDispatcher = nullptr;
		std::unique_ptr<btBroadphaseInterface> m_btOverlappingPairCache = nullptr;
		std::unique_ptr<PhysOverlapFilterCallback> m_overlapFilterCallback;
		std::unique_ptr<btSequentialImpulseConstraintSolver> m_btSolver = nullptr;
		std::unique_ptr<btGhostPairCallback> m_btGhostPairCallback = nullptr;
		std::unique_ptr<btSoftBodySolver> m_softBodySolver = nullptr;
		std::unique_ptr<btSoftBodyWorldInfo> m_softBodyWorldInfo;

		void AddAction(btActionInterface *action);

		void SimulationCallback(double timeStep);
	};
};

#endif
