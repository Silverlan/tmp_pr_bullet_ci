#ifndef __PR_BT_COLLISION_OBJECT_HPP__
#define __PR_BT_COLLISION_OBJECT_HPP__

#include <pragma/physics/collision_object.hpp>
#include <BulletSoftBody/btSoftBody.h>

class btCollisionObject;
class btRigidBody;
class btSoftBody;
class btMotionState;
class btGhostObject;
class btPairCachingGhostObject;
namespace pragma::physics
{
	class IEnvironment;
	class BtEnvironment;
	class BtRigidBody;
	class BtSoftBody;
	class BtGhostObject;
	class BtCollisionObject
		: virtual public ICollisionObject
	{
	public:
		friend IEnvironment;
		btCollisionObject &GetInternalObject() const;
		virtual void Initialize(lua_State *l,const util::TWeakSharedHandle<IBase> &handle) override;
		btCollisionObject &GetBtCollisionObject();
		virtual BtRigidBody *GetBtRigidBody();
		const BtRigidBody *GetBtRigidBody() const;
		virtual BtSoftBody *GetBtSoftBody();
		const BtSoftBody *GetBtSoftBody() const;
		virtual BtGhostObject *GetBtGhostObject();
		const BtGhostObject *GetBtGhostObject() const;

		virtual void Spawn() override;
		int GetCollisionFlags() const;
		void SetCollisionFlags(int flags);

		virtual ActivationState GetActivationState() const override;
		virtual void SetActivationState(ActivationState state) override;

		virtual void RemoveWorldObject() override;

		virtual bool IsStatic() const override;
		virtual void SetStatic(bool b) override;

		virtual void WakeUp(bool forceActivation=false) override;
		virtual void PutToSleep() override;
		virtual bool IsAsleep() const override;
		virtual void SetContactProcessingThreshold(float threshold) override;
		virtual void ApplyCollisionShape(pragma::physics::IShape *optShape) override;
		virtual void DoSetCollisionFilterGroup(CollisionMask group) override;
		virtual void DoSetCollisionFilterMask(CollisionMask mask) override;

		virtual void SetCCDEnabled(bool b) override;
		virtual void GetAABB(Vector3 &min,Vector3 &max) const override;
		virtual Vector3 GetPos() const override;
		virtual void SetPos(const Vector3 &pos) override;
		virtual Quat GetRotation() const override;
		virtual void SetRotation(const Quat &rot) override;
		virtual Transform GetWorldTransform() override;
		virtual void SetWorldTransform(const Transform &t) override;

		virtual bool IsTrigger() override;

		virtual void SetSimulationEnabled(bool b) override;
		virtual bool IsSimulationEnabled() const override;
		virtual void SetCollisionsEnabled(bool enabled) override;
	protected:
		BtCollisionObject(IEnvironment &env,std::unique_ptr<btCollisionObject> o,IShape &shape);
		void UpdateCCD();
		BtEnvironment &GetBtEnv() const;
		virtual void DoAddWorldObject() override;
		std::unique_ptr<btCollisionObject> m_collisionObject = nullptr;
	};

	class BtShape;
	class BtRigidBody
		: public BtCollisionObject,
		public IRigidBody,
		public nwm::VelocityCorrection
	{
	public:
		friend IEnvironment;
		btRigidBody &GetInternalObject() const;
		virtual ~BtRigidBody() override;
		virtual BtRigidBody *GetBtRigidBody() override;
		virtual void SetPos(const Vector3 &pos) override;
		virtual Vector3 GetPos() const override;
		virtual Quat GetRotation() const override;
		virtual void SetRotation(const Quat &rot) override;
		virtual void ApplyForce(const Vector3 &force) override;
		virtual void ApplyForce(const Vector3 &force,const Vector3 &relPos) override;
		virtual void ApplyImpulse(const Vector3 &impulse) override;
		virtual void ApplyImpulse(const Vector3 &impulse,const Vector3 &relPos) override;
		virtual void ApplyTorque(const Vector3 &torque) override;
		virtual void ApplyTorqueImpulse(const Vector3 &torque) override;
		virtual void ClearForces() override;
		virtual Vector3 GetTotalForce() override;
		virtual Vector3 GetTotalTorque() override;
		virtual void SetMassProps(float mass,const Vector3 &inertia) override;
		virtual float GetMass() const override;
		virtual void SetMass(float mass) override;
		virtual Vector3 &GetInertia() override;
		virtual Mat3 GetInvInertiaTensorWorld() const override;
		virtual void SetInertia(const Vector3 &inertia) override;
		virtual Vector3 GetLinearVelocity() const override;
		virtual Vector3 GetAngularVelocity() const override;
		virtual void SetLinearVelocity(const Vector3 &vel) override;
		virtual void SetAngularVelocity(const Vector3 &vel) override;
		virtual void SetLinearFactor(const Vector3 &factor) override;
		virtual void SetAngularFactor(const Vector3 &factor) override;
		virtual Vector3 GetLinearFactor() const override;
		virtual Vector3 GetAngularFactor() const override;
		virtual void SetLinearDamping(float damping) override;
		virtual void SetAngularDamping(float damping) override;
		virtual float GetLinearDamping() const override;
		virtual float GetAngularDamping() const override;
		virtual void SetLinearSleepingThreshold(float threshold) override;
		virtual void SetAngularSleepingThreshold(float threshold) override;
		virtual float GetLinearSleepingThreshold() const override;
		virtual float GetAngularSleepingThreshold() const override;
		virtual bool IsStatic() const override;
		virtual void SetStatic(bool b) override;

		virtual void SetKinematic(bool bKinematic) override;
		virtual bool IsKinematic() const override;

		virtual void PreSimulate() override;
		virtual void PostSimulate() override;
	protected:
		BtRigidBody(IEnvironment &env,std::unique_ptr<btRigidBody> body,float mass,IShape &shape,const Vector3 &localInertia);
		BtRigidBody(IEnvironment &env,float mass,BtShape &shape,const Vector3 &localInertia);
		struct KinematicData
		{
			Vector3 linearVelocity = {};
			Vector3 angularVelocity = {};
		};

		KinematicData m_kinematicData = {};

		float m_mass = 0.f;
		Vector3 m_inertia = {};
		std::unique_ptr<btMotionState> m_motionState = nullptr;
		virtual void DoAddWorldObject() override;
		virtual void RemoveWorldObject() override;
	};

	class BtSoftBody
		: public BtCollisionObject,
		public ISoftBody
	{
	public:
		friend IEnvironment;
		btSoftBody &GetInternalObject() const;
		virtual BtSoftBody *GetBtSoftBody() override;
		const btAlignedObjectArray<btSoftBody::Node> &GetNodes() const;

		virtual void AddVelocity(const Vector3 &vel) override;

		virtual const std::vector<uint16_t> &GetMeshVertexIndicesToLocalIndices() const override;
		virtual const std::vector<uint16_t> &GetLocalVertexIndicesToNodeIndices() const override;
		virtual const std::vector<uint16_t> &GetLocalVertexIndicesToMeshVertexIndices() const override;
		virtual const std::vector<uint16_t> &GetNodeIndicesToLocalVertexIndices() const override;

		virtual bool MeshVertexIndexToLocalVertexIndex(uint16_t meshVertexIndex,uint16_t &localIndex) const override;
		virtual bool LocalVertexIndexToMeshVertexIndex(uint16_t localIndex,uint16_t &meshVertexIndex) const override;
		virtual bool LocalVertexIndexToNodeIndex(uint16_t localVertexIndex,uint16_t &nodeIndex) const override;
		virtual bool NodeIndexToLocalVertexIndex(uint16_t nodeIndex,uint16_t &localVertexIndex) const override;

		virtual bool MeshVertexIndexToNodeIndex(uint16_t meshVertexIndex,uint16_t &nodeIndex) const override;
		virtual bool NodeIndexToMeshVertexIndex(uint16_t nodeIndex,uint16_t &meshVertexIndex) const override;

		virtual void SetSubMesh(const ModelSubMesh &subMesh,const std::vector<uint16_t> &meshVertexIndicesToLocalVertexIndices) override;

		virtual Vector3 GetPos() const override;
		virtual void SetPos(const Vector3 &pos) override;
		virtual void SetRotation(const Quat &rot) override;
		virtual Quat GetRotation() const override;
		virtual void SetWorldTransform(const Transform &t) override;

		virtual void UpdateLinearVelocity() override;

		virtual void AppendAnchor(uint32_t nodeId,IRigidBody &body,const Vector3 &localPivot,bool bDisableCollision=false,float influence=1.f) override;
		virtual void AppendAnchor(uint32_t nodeId,IRigidBody &body,bool bDisableCollision=false,float influence=1.f) override;
		virtual uint32_t GetNodeCount() const override;

		virtual const Vector3 &GetLinearVelocity() const override;

		virtual void GetAABB(Vector3 &min,Vector3 &max) const override;
		virtual void AddAeroForceToNode(int32_t node,const Vector3 &force) override;
		virtual void AddAeroForceToFace(int32_t face,const Vector3 &force) override;
		virtual void AddForce(const Vector3 &force) override;
		virtual void AddForce(uint32_t node,const Vector3 &force) override;
		virtual void AddLinearVelocity(const Vector3 &vel) override;
		virtual void AddLinearVelocity(uint32_t node,const Vector3 &vel) override;
		virtual float GetFriction() const override;
		virtual float GetHitFraction() const override;
		virtual float GetRollingFriction() const override;
		virtual Vector3 GetAnisotropicFriction() const override;
		virtual void SetFriction(float friction) override;
		virtual void SetHitFraction(float fraction) override;
		virtual void SetRollingFriction(float friction) override;
		virtual void SetAnisotropicFriction(const Vector3 &friction) override;
		virtual float GetMass(int32_t node) const override;
		virtual float GetMass() const override;
		virtual float GetRestitution() const override;
		virtual float GetRestLengthScale() const override;
		virtual Vector3 GetWindVelocity() const override;
		virtual void SetMass(int32_t node,float mass) override;
		virtual void SetMass(float mass) override;
		virtual void SetRestitution(float rest) override;
		virtual void SetRestLengthScale(float scale) override;
		virtual void SetWindVelocity(const Vector3 &vel) override;
		virtual void SetLinearVelocity(const Vector3 &vel) override;
		virtual void SetVolumeDensity(float density) override;
		virtual void SetVolumeMass(float mass) override;
		virtual float GetVolume() const override;
		virtual void SetDensity(float density) override;

		virtual void SetAnchorsHardness(float val) override;
		virtual void SetRigidContactsHardness(float val) override;
		virtual void SetDynamicFrictionCoefficient(float val) override;
		virtual void SetDragCoefficient(float val) override;
		virtual void SetDampingCoefficient(float val) override;
		virtual void SetKineticContactsHardness(float val) override;
		virtual void SetLiftCoefficient(float val) override;
		virtual void SetPoseMatchingCoefficient(float val) override;
		virtual void SetPressureCoefficient(float val) override;
		virtual void SetSoftContactsHardness(float val) override;
		virtual void SetSoftVsKineticHardness(float val) override;
		virtual void SetSoftVsRigidImpulseSplitK(float val) override;
		virtual void SetSoftVsRigidHardness(float val) override;
		virtual void SetSoftVsRigidImpulseSplitR(float val) override;
		virtual void SetSoftVsSoftHardness(float val) override;
		virtual void SetSoftVsRigidImpulseSplitS(float val) override;
		virtual void SetVolumeConversationCoefficient(float val) override;
		virtual void SetVelocitiesCorrectionFactor(float val) override;

		virtual float GetAnchorsHardness() const override;
		virtual float GetRigidContactsHardness() const override;
		virtual float GetDynamicFrictionCoefficient() const override;
		virtual float GetDragCoefficient() const override;
		virtual float GetDampingCoefficient() const override;
		virtual float GetKineticContactsHardness() const override;
		virtual float GetLiftCoefficient() const override;
		virtual float GetPoseMatchingCoefficient() const override;
		virtual float GetPressureCoefficient() const override;
		virtual float GetSoftContactsHardness() const override;
		virtual float GetSoftVsKineticHardness() const override;
		virtual float GetSoftVsRigidImpulseSplitK() const override;
		virtual float GetSoftVsRigidHardness() const override;
		virtual float GetSoftVsRigidImpulseSplitR() const override;
		virtual float GetSoftVsSoftHardness() const override;
		virtual float GetSoftVsRigidImpulseSplitS() const override;
		virtual float GetVolumeConversationCoefficient() const override;
		virtual float GetVelocitiesCorrectionFactor() const override;

		virtual void SetMaterialAngularStiffnessCoefficient(uint32_t matId,float val) override;
		virtual void SetMaterialLinearStiffnessCoefficient(uint32_t matId,float val) override;
		virtual void SetMaterialVolumeStiffnessCoefficient(uint32_t matId,float val) override;
		virtual float GetMaterialAngularStiffnessCoefficient(uint32_t matId) const override;
		virtual float GetMaterialLinearStiffnessCoefficient(uint32_t matId) const override;
		virtual float GetMaterialVolumeStiffnessCoefficient(uint32_t matId) const override;
	protected:
		BtSoftBody(IEnvironment &env,std::unique_ptr<btSoftBody> o,IShape &shape,const std::vector<uint16_t> &meshVertIndicesToPhysIndices);
		virtual void DoAddWorldObject() override;

		Quat m_rotation = uquat::identity();
		float m_totalMass = 0.f;
		Vector3 m_linearVelocity = {};
		std::vector<uint16_t> m_meshVertexIndicesToLocalVertexIndices;
		std::vector<uint16_t> m_localVertexIndicesToMeshVertexIndices;

		std::vector<uint16_t> m_localVertexIndicesToNodeIndices;
		std::vector<uint16_t> m_nodeIndicesToLocalVertexIndices;
		virtual void RemoveWorldObject() override;
		void UpdateTotalMass();
	};

	class BtGhostObject
		: public BtCollisionObject,
		public IGhostObject
	{
	public:
		friend IEnvironment;
		btPairCachingGhostObject &GetInternalObject() const;
		virtual BtGhostObject *GetBtGhostObject() override;
	protected:
		BtGhostObject(IEnvironment &env,std::unique_ptr<btPairCachingGhostObject> o,IShape &shape);

		virtual void DoAddWorldObject() override;
		virtual void RemoveWorldObject() override;
	};
};

#endif
