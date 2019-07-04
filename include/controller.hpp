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
	class BtController
		: virtual public IController
	{
	public:
		friend IEnvironment;

		virtual void Move(Vector3 &disp) override;
		virtual Vector3 GetDimensions() const override;
		virtual void SetDimensions(const Vector3 &dimensions) override;

		PhysKinematicCharacterController *GetCharacterController();
		const pragma::physics::BtConvexShape *GetShape() const;
		pragma::physics::BtConvexShape *GetShape();
	protected:
		BtController(IEnvironment &env,IGhostObject &ghostObject,IConvexShape &shape,std::unique_ptr<PhysKinematicCharacterController> controller);
		BtEnvironment &GetBtEnv() const;
		std::unique_ptr<PhysKinematicCharacterController> m_controller = nullptr;

		std::shared_ptr<pragma::physics::BtConvexShape> m_shape = nullptr;
	};
};

#endif
