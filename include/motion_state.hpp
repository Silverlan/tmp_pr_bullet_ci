#ifndef __PHYS_KINEMATIC_MOTION_STATE_HPP__
#define __PHYS_KINEMATIC_MOTION_STATE_HPP__

#include <mathutil/transform.hpp>
#include <LinearMath/btMotionState.h>

namespace pragma::physics {class BtCollisionObject;};
class SimpleMotionState
	: public btMotionState
{
public:
	SimpleMotionState(pragma::physics::BtCollisionObject &o);
	virtual void getWorldTransform(btTransform &worldTrans) const override;
	virtual void setWorldTransform(const btTransform &worldTrans) override;
	pragma::physics::BtCollisionObject &collisionObject;
};

class KinematicMotionState
	: public SimpleMotionState
{
public:
	KinematicMotionState(pragma::physics::BtCollisionObject &o,const umath::Transform &initialTransform={});
	virtual ~KinematicMotionState() override;

	umath::Transform &GetWorldTransform();
	const umath::Transform &GetWorldTransform() const;
private:
	virtual void getWorldTransform(btTransform &worldTrans) const override;
	virtual void setWorldTransform(const btTransform &worldTrans) override;
	umath::Transform m_transform = {};
};

#endif
