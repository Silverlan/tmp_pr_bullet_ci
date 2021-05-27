#ifndef __PHYS_KINEMATIC_MOTION_STATE_HPP__
#define __PHYS_KINEMATIC_MOTION_STATE_HPP__

#include <mathutil/transform.hpp>
#include <LinearMath/btMotionState.h>

class KinematicMotionState
	: public btMotionState
{
public:
	KinematicMotionState(const umath::Transform &initialTransform={});
	virtual ~KinematicMotionState() override;

	umath::Transform &GetWorldTransform();
	const umath::Transform &GetWorldTransform() const;
private:
	virtual void getWorldTransform(btTransform &worldTrans) const override;
	virtual void setWorldTransform(const btTransform &worldTrans) override;
	umath::Transform m_transform = {};
};

#endif
