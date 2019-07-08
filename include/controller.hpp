#ifndef __PR_BT_CONTROLLER_HPP__
#define __PR_BT_CONTROLLER_HPP__

#include <pragma/physics/controller.hpp>
#include <mathutil/uvec.h>

class PhysKinematicCharacterController;
class btCollisionObject;
namespace pragma::physics
{
	class BtEnvironment;
	class IShape;
	class IGhostObject;
	class BtConvexShape;
	class BtController
		: virtual public IController
	{
	public:
		friend IEnvironment;
		friend BtEnvironment;
		virtual Vector3 GetDimensions() const override;
		virtual void SetDimensions(const Vector3 &dimensions) override;
		virtual void Resize(float newHeight) override;
		virtual void SetFootPos(const Vector3 &footPos) override;
		virtual Vector3 GetFootPos() const override;

		PhysKinematicCharacterController *GetCharacterController();
		const pragma::physics::BtConvexShape *GetShape() const;
		pragma::physics::BtConvexShape *GetShape();
	protected:
		BtController(IEnvironment &env,const util::TSharedHandle<IGhostObject> &ghostObject,std::unique_ptr<PhysKinematicCharacterController> controller);
		virtual CollisionFlags DoMove(Vector3 &disp) override;
		BtEnvironment &GetBtEnv() const;
		std::unique_ptr<PhysKinematicCharacterController> m_controller = nullptr;

		std::shared_ptr<pragma::physics::BtConvexShape> m_shape = nullptr;
	};
};

#endif
