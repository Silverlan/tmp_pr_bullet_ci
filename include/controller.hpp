/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef __PR_BT_CONTROLLER_HPP__
#define __PR_BT_CONTROLLER_HPP__

#include <pragma/physics/controller.hpp>
#include <mathutil/uvec.h>
#include <BulletCollision/NarrowPhaseCollision/btManifoldPoint.h>

class PhysKinematicCharacterController;
class btCollisionObject;
class btManifoldPoint;
namespace pragma::physics
{
	class BtEnvironment;
	class IShape;
	class IGhostObject;
	class BtConvexShape;

	struct PhysContactInfo
	{
		static Vector3 GetContactNormal(const Vector3 &n,int8_t controllerIndex);
		static double CalcXZDistance(const btManifoldPoint &contactPoint,int8_t controllerIndex);

		PhysContactInfo(const btManifoldPoint &contactPoint,int8_t controllerIndex);
		Vector3 GetContactNormal() const;
		double CalcXZDistance() const;
		btManifoldPoint contactPoint {};
		util::TWeakSharedHandle<ICollisionObject> contactObject0 {};
		util::TWeakSharedHandle<ICollisionObject> contactObject1 {};
		std::weak_ptr<IShape> contactShape0 {};
		std::weak_ptr<IShape> contactShape1 {};
		int32_t surfaceMaterialId = -1;
		int8_t controllerIndex = -1; // = 0 if the controller is object 0 in 'contactPoint', otherwise 1
	};

	struct GroundInfo
	{
		GroundInfo(const btManifoldPoint &contactPoint,int8_t controllerIndex)
			: contactInfo{contactPoint,controllerIndex}
		{}
		PhysContactInfo contactInfo;
		bool groundWalkable = false;
		double contactDistance = std::numeric_limits<double>::max(); // Distance on XZ plane; Used to determine best contact point candidate
		double minContactDistance = std::numeric_limits<double>::max(); // Minimum XZ distance for ALL contact points (in this tick)
	};

	class BtController
		: virtual public IController
	{
	public:
		friend IEnvironment;
		friend BtEnvironment;

		virtual CollisionFlags GetCollisionFlags() const override;
		virtual IShape *GetGroundShape() const override;
		virtual IRigidBody *GetGroundBody() const override;
		virtual IMaterial *GetGroundMaterial() const override;
		virtual bool IsTouchingGround() const override;
		virtual std::optional<Vector3> GetGroundTouchPos() const override;
		virtual std::optional<Vector3> GetGroundTouchNormal() const override;
		virtual Vector3 GetDimensions() const override;
		virtual void SetDimensions(const Vector3 &dimensions) override;
		virtual void Resize(float newHeight) override;
		virtual void SetPos(const Vector3 &pos) override;
		virtual Vector3 GetPos() const override;
		virtual void SetFootPos(const Vector3 &footPos) override;
		virtual Vector3 GetFootPos() const override;
		virtual void SetUpDirection(const Vector3 &up) override;
		virtual Vector3 GetUpDirection() const override;

		virtual void SetSlopeLimit(umath::Degree slopeLimit) override;
		virtual umath::Degree GetSlopeLimit() const override;

		virtual void SetStepHeight(float stepHeight) override;
		virtual float GetStepHeight() const override;

		virtual Vector3 GetLinearVelocity() const override;
		virtual void SetLinearVelocity(const Vector3 &vel) override;

		void PreSimulate(float dt);
		void PostSimulate(float dt);

		PhysKinematicCharacterController *GetCharacterController();
		const pragma::physics::BtConvexShape *GetShape() const;
		pragma::physics::BtConvexShape *GetShape();

		bool SetGroundContactPoint(const btManifoldPoint &contactPoint,int32_t idx,const btCollisionObject *o,const btCollisionObject *oOther);
		void ClearGroundContactPoint();
	protected:
		BtController(IEnvironment &env,const std::shared_ptr<BtConvexShape> &shape,const util::TSharedHandle<IGhostObject> &ghostObject,std::unique_ptr<PhysKinematicCharacterController> controller,const Vector3 &halfExtents,ShapeType shapeType);
		virtual void RemoveWorldObject() override;
		virtual void DoAddWorldObject() override;
		virtual void DoMove(Vector3 &disp) override;
		BtEnvironment &GetBtEnv() const;
		std::unique_ptr<PhysKinematicCharacterController> m_controller = nullptr;

		std::optional<GroundInfo> m_groundInfo {};
		Vector3 m_velocity {};
		Vector3 m_preSimulationPosition {};
		std::shared_ptr<pragma::physics::BtConvexShape> m_shape = nullptr;
	};
};

#endif
