/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "constraint.hpp"
#include "environment.hpp"
#include "collision_object.hpp"
#include <pragma/networkstate/networkstate.h>
#include <pragma/game/game_coordinate_system.hpp>

#pragma optimize("",off)
pragma::physics::BtConstraint::BtConstraint(pragma::physics::IEnvironment &env,std::unique_ptr<btTypedConstraint> c)
	: IConstraint{env},m_constraint{std::move(c)}
{}

pragma::physics::BtEnvironment &pragma::physics::BtConstraint::GetBtEnv() const {return static_cast<BtEnvironment&>(m_physEnv);}
btTypedConstraint &pragma::physics::BtConstraint::GetInternalObject() const {return *m_constraint;}

void pragma::physics::BtConstraint::Initialize()
{
	IConstraint::Initialize();
	auto &physEnv = GetBtEnv();
	auto *world = physEnv.GetWorld();
	world->removeConstraint(m_constraint.get());
	world->addConstraint(m_constraint.get());
}

pragma::physics::IRigidBody *pragma::physics::BtConstraint::GetSourceActor()
{
	auto &bodyA = m_constraint->getRigidBodyA();
	return static_cast<pragma::physics::ICollisionObject*>(bodyA.getUserPointer())->GetRigidBody();
}
pragma::physics::IRigidBody *pragma::physics::BtConstraint::GetTargetActor()
{
	auto &bodyB = m_constraint->getRigidBodyB();
	return static_cast<pragma::physics::ICollisionObject*>(bodyB.getUserPointer())->GetRigidBody();
}

void pragma::physics::BtConstraint::DoSetCollisionsEnabled(Bool b)
{
	auto &bodyA = m_constraint->getRigidBodyA();
	auto &bodyB = m_constraint->getRigidBodyB();
	if(b == false)
	{
		bodyA.addConstraintRef(m_constraint.get());
		bodyB.addConstraintRef(m_constraint.get());
		return;
	}
	bodyA.removeConstraintRef(m_constraint.get());
	bodyB.removeConstraintRef(m_constraint.get());
}

void pragma::physics::BtConstraint::RemoveWorldObject()
{
	// TODO
}
void pragma::physics::BtConstraint::DoAddWorldObject()
{
	// TODO
}

void pragma::physics::BtConstraint::SetEnabled(bool b) {m_constraint->setEnabled(b);}
bool pragma::physics::BtConstraint::IsEnabled() const {return m_constraint->isEnabled();}

btTypedConstraint &pragma::physics::BtConstraint::GetBtConstraint() const {return *m_constraint;}

bool pragma::physics::BtConstraint::IsBroken() const
{
	// TODO
	return false;
}
void pragma::physics::BtConstraint::Break()
{
	// TODO
}

float pragma::physics::BtConstraint::GetBreakForce() const
{
	// TODO
	return 0.f;
}
void pragma::physics::BtConstraint::SetBreakForce(float threshold)
{
	// TODO
}
float pragma::physics::BtConstraint::GetBreakTorque() const
{
	// TODO
	return 0.f;
}
void pragma::physics::BtConstraint::SetBreakTorque(float torque)
{
	// TODO
}

void pragma::physics::BtConstraint::SetSoftness(float softness)
{
	// TODO
}
void pragma::physics::BtConstraint::SetDamping(float damping)
{
	// TODO
}
void pragma::physics::BtConstraint::SetRestitution(float restitution)
{
	// TODO
}

float pragma::physics::BtConstraint::GetSoftness() const
{
	// TODO
	return 0.f;
}
float pragma::physics::BtConstraint::GetDamping() const
{
	// TODO
	return 0.f;
}
float pragma::physics::BtConstraint::GetRestitution() const
{
	// TODO
	return 0.f;
}

pragma::physics::BtFixedConstraint *pragma::physics::BtConstraint::GetBtFixedConstraint() {return nullptr;}
const pragma::physics::BtFixedConstraint *pragma::physics::BtConstraint::GetBtFixedConstraint() const {return const_cast<BtConstraint*>(this)->GetBtFixedConstraint();}
pragma::physics::BtBallSocketConstraint *pragma::physics::BtConstraint::GetBtBallSocketConstraint() {return nullptr;}
const pragma::physics::BtBallSocketConstraint *pragma::physics::BtConstraint::GetBtBallSocketConstraint() const {return const_cast<BtConstraint*>(this)->GetBtBallSocketConstraint();}
pragma::physics::BtHingeConstraint *pragma::physics::BtConstraint::GetBtHingeConstraint() {return nullptr;}
const pragma::physics::BtHingeConstraint *pragma::physics::BtConstraint::GetBtHingeConstraint() const {return const_cast<BtConstraint*>(this)->GetBtHingeConstraint();}
pragma::physics::BtSliderConstraint *pragma::physics::BtConstraint::GetBtSliderConstraint() {return nullptr;}
const pragma::physics::BtSliderConstraint *pragma::physics::BtConstraint::GetBtSliderConstraint() const {return const_cast<BtConstraint*>(this)->GetBtSliderConstraint();}
pragma::physics::BtConeTwistConstraint *pragma::physics::BtConstraint::GetBtConeTwistConstraint() {return nullptr;}
const pragma::physics::BtConeTwistConstraint *pragma::physics::BtConstraint::GetBtConeTwistConstraint() const {return const_cast<BtConstraint*>(this)->GetBtConeTwistConstraint();}
pragma::physics::BtDoFConstraint *pragma::physics::BtConstraint::GetBtDoFConstraint() {return nullptr;}
const pragma::physics::BtDoFConstraint *pragma::physics::BtConstraint::GetBtDoFConstraint() const {return const_cast<BtConstraint*>(this)->GetBtDoFConstraint();}
pragma::physics::BtDoFSpringConstraint *pragma::physics::BtConstraint::GetBtDoFSpringConstraint() {return nullptr;}
const pragma::physics::BtDoFSpringConstraint *pragma::physics::BtConstraint::GetBtDoFSpringConstraint() const {return const_cast<BtConstraint*>(this)->GetBtDoFSpringConstraint();}

/*void pragma::physics::BtConstraint::SetOverrideSolverIterationCount(int32_t count) {m_constraint->setOverrideNumSolverIterations(count);}
int32_t pragma::physics::BtConstraint::GetOverrideSolverIterationCount() const {return m_constraint->getOverrideNumSolverIterations();}
float pragma::physics::BtConstraint::GetBreakingImpulseThreshold() const {return m_constraint->getBreakingImpulseThreshold() /BtEnvironment::WORLD_SCALE;}
void pragma::physics::BtConstraint::SetBreakingImpulseThreshold(float threshold) {m_constraint->setBreakingImpulseThreshold(threshold *BtEnvironment::WORLD_SCALE);}*/

////////////////////////////

pragma::physics::BtFixedConstraint::BtFixedConstraint(IEnvironment &env,std::unique_ptr<btFixedConstraint> constraint)
	: IFixedConstraint{env},BtConstraint{env,std::move(constraint)},IConstraint{env}
{
	auto &c = GetInternalObject();
	m_srcTransform = GetBtEnv().CreateTransform(c.getFrameOffsetA());
	m_tgtTransform = GetBtEnv().CreateTransform(c.getFrameOffsetB());
}
pragma::physics::BtFixedConstraint *pragma::physics::BtFixedConstraint::GetBtFixedConstraint() {return this;}
btFixedConstraint &pragma::physics::BtFixedConstraint::GetInternalObject() const {return static_cast<btFixedConstraint&>(BtConstraint::GetInternalObject());}

////////////////////////////

pragma::physics::BtBallSocketConstraint::BtBallSocketConstraint(IEnvironment &env,std::unique_ptr<btPoint2PointConstraint> constraint)
	: IBallSocketConstraint{env},BtConstraint{env,std::move(constraint)},IConstraint{env}
{}
pragma::physics::BtBallSocketConstraint *pragma::physics::BtBallSocketConstraint::GetBtBallSocketConstraint() {return this;}
btPoint2PointConstraint &pragma::physics::BtBallSocketConstraint::GetInternalObject() const {return static_cast<btPoint2PointConstraint&>(BtConstraint::GetInternalObject());}

////////////////////////////

pragma::physics::BtHingeConstraint::BtHingeConstraint(IEnvironment &env,std::unique_ptr<btHingeConstraint> constraint)
	: IHingeConstraint{env},BtConstraint{env,std::move(constraint)},IConstraint{env}
{}

pragma::physics::BtHingeConstraint *pragma::physics::BtHingeConstraint::GetBtHingeConstraint() {return this;}
btHingeConstraint &pragma::physics::BtHingeConstraint::GetInternalObject() const {return static_cast<btHingeConstraint&>(BtConstraint::GetInternalObject());}

void pragma::physics::BtHingeConstraint::SetLimit(umath::Radian lowerLimit,umath::Radian upperLimit)
{
	// TODO
}
std::pair<umath::Radian,umath::Radian> pragma::physics::BtHingeConstraint::GetLimit() const
{
	// TODO
	return {};
}
void pragma::physics::BtHingeConstraint::DisableLimit()
{
	// TODO
}

void pragma::physics::BtHingeConstraint::SetLimit(float low,float high,float softness,float biasFactor,float relaxationFactor)
{
	GetInternalObject().setLimit(low,high,softness,biasFactor,relaxationFactor);
}

////////////////////////////

pragma::physics::BtSliderConstraint::BtSliderConstraint(IEnvironment &env,std::unique_ptr<btSliderConstraint> constraint)
	: ISliderConstraint{env},BtConstraint{env,std::move(constraint)},IConstraint{env}
{}
btSliderConstraint &pragma::physics::BtSliderConstraint::GetInternalObject() const {return static_cast<btSliderConstraint&>(BtConstraint::GetInternalObject());}

pragma::physics::BtSliderConstraint *pragma::physics::BtSliderConstraint::GetBtSliderConstraint() {return this;}

void pragma::physics::BtSliderConstraint::SetLimit(float lowerLimit,float upperLimit)
{
	// TODO
}
void pragma::physics::BtSliderConstraint::DisableLimit()
{
	// TODO
}
std::pair<float,float> pragma::physics::BtSliderConstraint::GetLimit() const
{
	// TODO
	return {};
}

////////////////////////////

pragma::physics::BtConeTwistConstraint::BtConeTwistConstraint(IEnvironment &env,std::unique_ptr<btConeTwistConstraint> constraint)
	: IConeTwistConstraint{env},BtConstraint{env,std::move(constraint)},IConstraint{env}
{
	auto &c = GetInternalObject();
	m_srcTransform = GetBtEnv().CreateTransform(c.getAFrame());
	m_tgtTransform = GetBtEnv().CreateTransform(c.getBFrame());
}
pragma::physics::BtConeTwistConstraint *pragma::physics::BtConeTwistConstraint::GetBtConeTwistConstraint() {return this;}
btConeTwistConstraint &pragma::physics::BtConeTwistConstraint::GetInternalObject() const {return static_cast<btConeTwistConstraint&>(BtConstraint::GetInternalObject());}
void pragma::physics::BtConeTwistConstraint::SetLimit(const Vector3 &lowerLimits,const Vector3 &upperLimits)
{
	// TODO
}
void pragma::physics::BtConeTwistConstraint::SetLimit(float swingSpan1,float swingSpan2,float twistSpan)
{
	// TODO
}
void pragma::physics::BtConeTwistConstraint::GetLimit(float &outSwingSpan1,float &outSwingSpan2,float &outTwistSpan)
{
	// TODO
}
/*void pragma::physics::BtConeTwistConstraint::SetLimit(AxisType type,pragma::Axis axis,double lo,double hi)
{
	// TODO
}
void pragma::physics::BtConeTwistConstraint::SetLimitReversed(AxisType type,pragma::Axis axis,double lo,double hi)
{
	// TODO
}*/
/*void pragma::physics::BtConeTwistConstraint::SetLimit(float swingSpan1,float swingSpan2,float twistSpan,float softness,float biasFactor,float relaxationFactor)
{
	GetInternalObject().setLimit(swingSpan1,swingSpan2,twistSpan,softness,biasFactor,relaxationFactor);
}*/

////////////////////////////

pragma::physics::BtDoFConstraint::BtDoFConstraint(IEnvironment &env,std::unique_ptr<btGeneric6DofConstraint> constraint)
	: IDoFConstraint{env},BtConstraint{env,std::move(constraint)},IConstraint{env}
{}
pragma::physics::BtDoFConstraint *pragma::physics::BtDoFConstraint::GetBtDoFConstraint() {return this;}
btGeneric6DofConstraint &pragma::physics::BtDoFConstraint::GetInternalObject() const {return static_cast<btGeneric6DofConstraint&>(BtConstraint::GetInternalObject());}
void pragma::physics::BtDoFConstraint::SetLinearLimit(const Vector3 &lower,const Vector3 &upper)
{
	SetLinearLowerLimit(lower);
	SetLinearUpperLimit(upper);
}
void pragma::physics::BtDoFConstraint::SetLinearLimit(const Vector3 &lim) {SetLinearLimit(-lim,lim);}
void pragma::physics::BtDoFConstraint::SetLinearLowerLimit(const Vector3 &lim)
{
	GetInternalObject().setLinearLowerLimit(uvec::create_bt(lim) *BtEnvironment::WORLD_SCALE);
}
void pragma::physics::BtDoFConstraint::SetLinearUpperLimit(const Vector3 &lim)
{
	GetInternalObject().setLinearUpperLimit(uvec::create_bt(lim) *BtEnvironment::WORLD_SCALE);
}
void pragma::physics::BtDoFConstraint::SetAngularLimit(const EulerAngles &lower,const EulerAngles &upper)
{
	SetAngularLowerLimit(lower);
	SetAngularUpperLimit(upper);
}
void pragma::physics::BtDoFConstraint::SetAngularLimit(const EulerAngles &lim) {SetAngularLimit(-lim,lim);}
void pragma::physics::BtDoFConstraint::SetAngularLowerLimit(const EulerAngles &lim)
{
	GetInternalObject().setAngularLowerLimit(btVector3(umath::deg_to_rad(lim.p),umath::deg_to_rad(lim.y),umath::deg_to_rad(lim.r)));
}
void pragma::physics::BtDoFConstraint::SetAngularUpperLimit(const EulerAngles &lim)
{
	GetInternalObject().setAngularUpperLimit(btVector3(umath::deg_to_rad(lim.p),umath::deg_to_rad(lim.y),umath::deg_to_rad(lim.r)));
}

Vector3 pragma::physics::BtDoFConstraint::GetLinearLowerLimit() const
{
	btVector3 lowerLimit;
	GetInternalObject().getLinearLowerLimit(lowerLimit);
	return uvec::create(lowerLimit /BtEnvironment::WORLD_SCALE);
}
Vector3 pragma::physics::BtDoFConstraint::GetlinearUpperLimit() const
{
	btVector3 upperLimit;
	GetInternalObject().getLinearUpperLimit(upperLimit);
	return uvec::create(upperLimit /BtEnvironment::WORLD_SCALE);
}
EulerAngles pragma::physics::BtDoFConstraint::GetAngularLowerLimit() const
{
	btVector3 lowerLimit;
	GetInternalObject().getAngularLowerLimit(lowerLimit);
	return EulerAngles(umath::rad_to_deg(lowerLimit.x()),umath::rad_to_deg(lowerLimit.y()),umath::rad_to_deg(lowerLimit.z()));
}
EulerAngles pragma::physics::BtDoFConstraint::GetAngularUpperLimit() const
{
	btVector3 upperLimit;
	GetInternalObject().getAngularUpperLimit(upperLimit);
	return EulerAngles(umath::rad_to_deg(upperLimit.x()),umath::rad_to_deg(upperLimit.y()),umath::rad_to_deg(upperLimit.z()));
}

Vector3 pragma::physics::BtDoFConstraint::GetAngularTargetVelocity() const
{
	Vector3 r {};
	for(uint8_t axis=0;axis<3;++axis)
	{
		auto *motor = GetInternalObject().getRotationalLimitMotor(axis);
		if(motor == nullptr)
			continue;
		r[axis] = motor->m_targetVelocity;
	}
	return r;
}
void pragma::physics::BtDoFConstraint::SetAngularTargetVelocity(const Vector3 &vel) const
{
	for(uint8_t axis=0;axis<3;++axis)
	{
		auto *motor = GetInternalObject().getRotationalLimitMotor(axis);
		if(motor == nullptr)
			continue;
		motor->m_targetVelocity = vel[axis];
	}
}
Vector3 pragma::physics::BtDoFConstraint::GetAngularMaxMotorForce() const
{
	Vector3 r {};
	for(uint8_t axis=0;axis<3;++axis)
	{
		auto *motor = GetInternalObject().getRotationalLimitMotor(axis);
		if(motor == nullptr)
			continue;
		r[axis] = motor->m_maxMotorForce;
	}
	return r;
}
void pragma::physics::BtDoFConstraint::SetAngularMaxMotorForce(const Vector3 &force)
{
	for(uint8_t axis=0;axis<3;++axis)
	{
		auto *motor = GetInternalObject().getRotationalLimitMotor(axis);
		if(motor == nullptr)
			continue;
		motor->m_maxMotorForce = force[axis];
	}
}
Vector3 pragma::physics::BtDoFConstraint::GetAngularMaxLimitForce() const
{
	Vector3 r {};
	for(uint8_t axis=0;axis<3;++axis)
	{
		auto *motor = GetInternalObject().getRotationalLimitMotor(axis);
		if(motor == nullptr)
			continue;
		r[axis] = motor->m_maxLimitForce;
	}
	return r;
}
void pragma::physics::BtDoFConstraint::SetAngularMaxLimitForce(const Vector3 &force)
{
	for(uint8_t axis=0;axis<3;++axis)
	{
		auto *motor = GetInternalObject().getRotationalLimitMotor(axis);
		if(motor == nullptr)
			continue;
		motor->m_maxLimitForce = force[axis];
	}
}
Vector3 pragma::physics::BtDoFConstraint::GetAngularDamping() const
{
	Vector3 r {};
	for(uint8_t axis=0;axis<3;++axis)
	{
		auto *motor = GetInternalObject().getRotationalLimitMotor(axis);
		if(motor == nullptr)
			continue;
		r[axis] = motor->m_damping;
	}
	return r;
}
void pragma::physics::BtDoFConstraint::SetAngularDamping(const Vector3 &damping)
{
	for(uint8_t axis=0;axis<3;++axis)
	{
		auto *motor = GetInternalObject().getRotationalLimitMotor(axis);
		if(motor == nullptr)
			continue;
		motor->m_damping = damping[axis];
	}
}
Vector3 pragma::physics::BtDoFConstraint::GetAngularLimitSoftness() const
{
	Vector3 r {};
	for(uint8_t axis=0;axis<3;++axis)
	{
		auto *motor = GetInternalObject().getRotationalLimitMotor(axis);
		if(motor == nullptr)
			continue;
		r[axis] = motor->m_limitSoftness;
	}
	return r;
}
void pragma::physics::BtDoFConstraint::SetAngularLimitSoftness(const Vector3 &softness)
{
	for(uint8_t axis=0;axis<3;++axis)
	{
		auto *motor = GetInternalObject().getRotationalLimitMotor(axis);
		if(motor == nullptr)
			continue;
		motor->m_limitSoftness = softness[axis];
	}
}
Vector3 pragma::physics::BtDoFConstraint::GetAngularForceMixingFactor() const
{
	Vector3 r {};
	for(uint8_t axis=0;axis<3;++axis)
	{
		auto *motor = GetInternalObject().getRotationalLimitMotor(axis);
		if(motor == nullptr)
			continue;
		r[axis] = motor->m_normalCFM;
	}
	return r;
}
void pragma::physics::BtDoFConstraint::SetAngularForceMixingFactor(const Vector3 &factor)
{
	for(uint8_t axis=0;axis<3;++axis)
	{
		auto *motor = GetInternalObject().getRotationalLimitMotor(axis);
		if(motor == nullptr)
			continue;
		motor->m_normalCFM = factor[axis];
	}
}
Vector3 pragma::physics::BtDoFConstraint::GetAngularLimitErrorTolerance() const
{
	Vector3 r {};
	for(uint8_t axis=0;axis<3;++axis)
	{
		auto *motor = GetInternalObject().getRotationalLimitMotor(axis);
		if(motor == nullptr)
			continue;
		r[axis] = motor->m_stopERP;
	}
	return r;
}
void pragma::physics::BtDoFConstraint::SetAngularLimitErrorTolerance(const Vector3 &tolerance)
{
	for(uint8_t axis=0;axis<3;++axis)
	{
		auto *motor = GetInternalObject().getRotationalLimitMotor(axis);
		if(motor == nullptr)
			continue;
		motor->m_stopERP = tolerance[axis];
	}
}
Vector3 pragma::physics::BtDoFConstraint::GetAngularLimitForceMixingFactor() const
{
	Vector3 r {};
	for(uint8_t axis=0;axis<3;++axis)
	{
		auto *motor = GetInternalObject().getRotationalLimitMotor(axis);
		if(motor == nullptr)
			continue;
		r[axis] = motor->m_stopCFM;
	}
	return r;
}
void pragma::physics::BtDoFConstraint::SetAngularLimitForceMixingFactor(const Vector3 &factor)
{
	for(uint8_t axis=0;axis<3;++axis)
	{
		auto *motor = GetInternalObject().getRotationalLimitMotor(axis);
		if(motor == nullptr)
			continue;
		motor->m_stopCFM = factor[axis];
	}
}
Vector3 pragma::physics::BtDoFConstraint::GetAngularRestitutionFactor() const
{
	Vector3 r {};
	for(uint8_t axis=0;axis<3;++axis)
	{
		auto *motor = GetInternalObject().getRotationalLimitMotor(axis);
		if(motor == nullptr)
			continue;
		r[axis] = motor->m_bounce;
	}
	return r;
}
void pragma::physics::BtDoFConstraint::SetAngularRestitutionFactor(const Vector3 &factor)
{
	for(uint8_t axis=0;axis<3;++axis)
	{
		auto *motor = GetInternalObject().getRotationalLimitMotor(axis);
		if(motor == nullptr)
			continue;
		motor->m_bounce = factor[axis];
	}
}
bool pragma::physics::BtDoFConstraint::IsAngularMotorEnabled(uint8_t axis) const
{
	auto *motor = GetInternalObject().getRotationalLimitMotor(axis);
	if(motor == nullptr)
		return 0.f;
	return motor->m_enableMotor;
}
void pragma::physics::BtDoFConstraint::SetAngularMotorEnabled(uint8_t axis,bool bEnabled)
{
	auto *motor = GetInternalObject().getRotationalLimitMotor(axis);
	if(motor == nullptr)
		return;
	motor->m_enableMotor = bEnabled;
}
Vector3 pragma::physics::BtDoFConstraint::GetCurrentAngularLimitError() const
{
	Vector3 r {};
	for(uint8_t axis=0;axis<3;++axis)
	{
		auto *motor = GetInternalObject().getRotationalLimitMotor(axis);
		if(motor == nullptr)
			continue;
		r[axis] = motor->m_currentLimitError;
	}
	return r;
}
Vector3 pragma::physics::BtDoFConstraint::GetCurrentAngularPosition() const
{
	Vector3 r {};
	for(uint8_t axis=0;axis<3;++axis)
	{
		auto *motor = GetInternalObject().getRotationalLimitMotor(axis);
		if(motor == nullptr)
			continue;
		r[axis] = motor->m_currentPosition;
	}
	return r;
}
Vector3i pragma::physics::BtDoFConstraint::GetCurrentAngularLimit() const
{
	Vector3 r {};
	for(uint8_t axis=0;axis<3;++axis)
	{
		auto *motor = GetInternalObject().getRotationalLimitMotor(axis);
		if(motor == nullptr)
			continue;
		r[axis] = motor->m_currentLimit;
	}
	return r;
}
Vector3 pragma::physics::BtDoFConstraint::GetCurrentAngularAccumulatedImpulse() const
{
	Vector3 r {};
	for(uint8_t axis=0;axis<3;++axis)
	{
		auto *motor = GetInternalObject().getRotationalLimitMotor(axis);
		if(motor == nullptr)
			continue;
		r[axis] = motor->m_accumulatedImpulse /BtEnvironment::WORLD_SCALE;
	}
	return r;
}

Vector3 pragma::physics::BtDoFConstraint::GetLinearTargetVelocity() const
{
	auto *motor = GetInternalObject().getTranslationalLimitMotor();
	if(motor == nullptr)
		return {};
	return uvec::create(motor->m_targetVelocity /BtEnvironment::WORLD_SCALE);
}
void pragma::physics::BtDoFConstraint::SetLinearTargetVelocity(const Vector3 &vel) const
{
	auto *motor = GetInternalObject().getTranslationalLimitMotor();
	if(motor == nullptr)
		return;
	motor->m_targetVelocity = uvec::create_bt(vel) *BtEnvironment::WORLD_SCALE;
}
Vector3 pragma::physics::BtDoFConstraint::GetLinearMaxMotorForce() const
{
	auto *motor = GetInternalObject().getTranslationalLimitMotor();
	if(motor == nullptr)
		return {};
	return uvec::create(motor->m_maxMotorForce /BtEnvironment::WORLD_SCALE);
}
void pragma::physics::BtDoFConstraint::SetLinearMaxMotorForce(const Vector3 &force)
{
	auto *motor = GetInternalObject().getTranslationalLimitMotor();
	if(motor == nullptr)
		return;
	motor->m_maxMotorForce = uvec::create_bt(force) *BtEnvironment::WORLD_SCALE;
}
float pragma::physics::BtDoFConstraint::GetLinearDamping() const
{
	auto *motor = GetInternalObject().getTranslationalLimitMotor();
	if(motor == nullptr)
		return 0.f;
	return motor->m_damping;
}
void pragma::physics::BtDoFConstraint::SetLinearDamping(float damping)
{
	auto *motor = GetInternalObject().getTranslationalLimitMotor();
	if(motor == nullptr)
		return;
	motor->m_damping = damping;
}
float pragma::physics::BtDoFConstraint::GetLinearLimitSoftness() const
{
	auto *motor = GetInternalObject().getTranslationalLimitMotor();
	if(motor == nullptr)
		return 0.f;
	return motor->m_limitSoftness;
}
void pragma::physics::BtDoFConstraint::SetLinearLimitSoftness(float softness)
{
	auto *motor = GetInternalObject().getTranslationalLimitMotor();
	if(motor == nullptr)
		return;
	motor->m_limitSoftness = softness;
}
Vector3 pragma::physics::BtDoFConstraint::GetLinearForceMixingFactor() const
{
	auto *motor = GetInternalObject().getTranslationalLimitMotor();
	if(motor == nullptr)
		return {};
	return uvec::create(motor->m_normalCFM /BtEnvironment::WORLD_SCALE);
}
void pragma::physics::BtDoFConstraint::SetLinearForceMixingFactor(const Vector3 &factor)
{
	auto *motor = GetInternalObject().getTranslationalLimitMotor();
	if(motor == nullptr)
		return;
	motor->m_normalCFM = uvec::create_bt(factor) *BtEnvironment::WORLD_SCALE;
}
Vector3 pragma::physics::BtDoFConstraint::GetLinearLimitErrorTolerance() const
{
	auto *motor = GetInternalObject().getTranslationalLimitMotor();
	if(motor == nullptr)
		return {};
	return uvec::create(motor->m_stopERP /BtEnvironment::WORLD_SCALE);
}
void pragma::physics::BtDoFConstraint::SetLinearLimitErrorTolerance(const Vector3 &tolerance)
{
	auto *motor = GetInternalObject().getTranslationalLimitMotor();
	if(motor == nullptr)
		return;
	motor->m_stopERP = uvec::create_bt(tolerance) *BtEnvironment::WORLD_SCALE;
}
Vector3 pragma::physics::BtDoFConstraint::GetLinearLimitForceMixingFactor() const
{
	auto *motor = GetInternalObject().getTranslationalLimitMotor();
	if(motor == nullptr)
		return {};
	return uvec::create(motor->m_stopCFM /BtEnvironment::WORLD_SCALE);
}
void pragma::physics::BtDoFConstraint::SetLinearLimitForceMixingFactor(const Vector3 &factor)
{
	auto *motor = GetInternalObject().getTranslationalLimitMotor();
	if(motor == nullptr)
		return;
	motor->m_stopCFM = uvec::create_bt(factor) *BtEnvironment::WORLD_SCALE;
}
float pragma::physics::BtDoFConstraint::GetLinearRestitutionFactor() const
{
	auto *motor = GetInternalObject().getTranslationalLimitMotor();
	if(motor == nullptr)
		return 0.f;
	return motor->m_restitution;
}
void pragma::physics::BtDoFConstraint::SetLinearRestitutionFactor(float factor)
{
	auto *motor = GetInternalObject().getTranslationalLimitMotor();
	if(motor == nullptr)
		return;
	motor->m_restitution = factor;
}
bool pragma::physics::BtDoFConstraint::IsLinearMotorEnabled(uint8_t axis) const
{
	auto *motor = GetInternalObject().getTranslationalLimitMotor();
	if(motor == nullptr || axis > 2)
		return false;
	return motor->m_enableMotor[axis];
}
void pragma::physics::BtDoFConstraint::SetLinearMotorEnabled(uint8_t axis,bool bEnabled)
{
	auto *motor = GetInternalObject().getTranslationalLimitMotor();
	if(motor == nullptr || axis > 2)
		return;
	motor->m_enableMotor[axis] = bEnabled;
}
Vector3 pragma::physics::BtDoFConstraint::GetCurrentLinearDifference() const
{
	auto *motor = GetInternalObject().getTranslationalLimitMotor();
	if(motor == nullptr)
		return {};
	return uvec::create(motor->m_currentLinearDiff /BtEnvironment::WORLD_SCALE);
}
Vector3 pragma::physics::BtDoFConstraint::GetCurrentLinearLimitError() const
{
	auto *motor = GetInternalObject().getTranslationalLimitMotor();
	if(motor == nullptr)
		return {};
	return uvec::create(motor->m_currentLimitError);
}
Vector3i pragma::physics::BtDoFConstraint::GetCurrentLinearLimit() const
{
	auto *motor = GetInternalObject().getTranslationalLimitMotor();
	if(motor == nullptr)
		return {};
	return {motor->m_currentLimit[0],motor->m_currentLimit[1],motor->m_currentLimit[2]};
}
Vector3 pragma::physics::BtDoFConstraint::GetCurrentLinearAccumulatedImpulse() const
{
	auto *motor = GetInternalObject().getTranslationalLimitMotor();
	if(motor == nullptr)
		return {};
	return uvec::create(motor->m_currentLinearDiff /BtEnvironment::WORLD_SCALE);
}

///////////////

pragma::physics::BtDoFSpringConstraint::BtDoFSpringConstraint(IEnvironment &env,std::unique_ptr<btGeneric6DofSpring2Constraint> constraint)
	: IDoFSpringConstraint{env},BtConstraint{env,std::move(constraint)},IConstraint{env}
{
	auto &c = GetInternalObject();
	m_srcTransform = GetBtEnv().CreateTransform(c.getFrameOffsetA());
	m_tgtTransform = GetBtEnv().CreateTransform(c.getFrameOffsetB());
}
btGeneric6DofSpring2Constraint &pragma::physics::BtDoFSpringConstraint::GetInternalObject() const {return static_cast<btGeneric6DofSpring2Constraint&>(BtConstraint::GetInternalObject());}
inline int32_t get_axis_index(pragma::physics::BtDoFSpringConstraint::AxisType type,pragma::Axis axis)
{
	auto r = umath::to_integral(axis);
	if(type == pragma::physics::BtDoFSpringConstraint::AxisType::Angular)
		r += 3u;
	return r;
}
inline RotateOrder get_spring_rotation_order(pragma::RotationOrder order)
{
	switch(order)
	{
	case pragma::RotationOrder::XYZ:
		return RO_XYZ;
	case pragma::RotationOrder::XZY:
		return RO_XZY;
	case pragma::RotationOrder::YXZ:
		return RO_YXZ;
	case pragma::RotationOrder::YZX:
		return RO_YZX;
	case pragma::RotationOrder::ZXY:
		return RO_ZXY;
	case pragma::RotationOrder::ZYX:
		return RO_ZYX;
	default: // Illegal case
		return RotateOrder::RO_XYZ;
	}
}
inline pragma::RotationOrder get_rotation_order(RotateOrder order)
{
	switch(order)
	{
	case RotateOrder::RO_XYZ:
		return pragma::RotationOrder::XYZ;
	case RotateOrder::RO_XZY:
		return pragma::RotationOrder::XZY;
	case RotateOrder::RO_YXZ:
		return pragma::RotationOrder::YXZ;
	case RotateOrder::RO_YZX:
		return pragma::RotationOrder::YZX;
	case RotateOrder::RO_ZXY:
		return pragma::RotationOrder::ZXY;
	case RotateOrder::RO_ZYX:
		return pragma::RotationOrder::ZYX;
	default:
		return pragma::RotationOrder::XYZ;
	}
}
pragma::physics::BtDoFSpringConstraint *pragma::physics::BtDoFSpringConstraint::GetBtDoFSpringConstraint() {return this;}
void pragma::physics::BtDoFSpringConstraint::CalculateTransforms()
{
	GetInternalObject().calculateTransforms();
}
void pragma::physics::BtDoFSpringConstraint::CalculateTransforms(const umath::Transform &frameA,const umath::Transform &frameB)
{
	GetInternalObject().calculateTransforms(GetBtEnv().CreateBtTransform(frameA),GetBtEnv().CreateBtTransform(frameB));
}
btRotationalLimitMotor2 *pragma::physics::BtDoFSpringConstraint::GetRotationalLimitMotor(pragma::Axis index) const
{
	return GetInternalObject().getRotationalLimitMotor(umath::to_integral(index));
}
btTranslationalLimitMotor2 *pragma::physics::BtDoFSpringConstraint::GetTranslationalLimitMotor() const
{
	return GetInternalObject().getTranslationalLimitMotor();
}
umath::Transform pragma::physics::BtDoFSpringConstraint::GetCalculatedTransformA() const
{
	return GetBtEnv().CreateTransform(GetInternalObject().getCalculatedTransformA());
}
umath::Transform pragma::physics::BtDoFSpringConstraint::GetCalculatedTransformB() const
{
	return GetBtEnv().CreateTransform(GetInternalObject().getCalculatedTransformB());
}
umath::Transform pragma::physics::BtDoFSpringConstraint::GetFrameOffsetA() const
{
	return GetBtEnv().CreateTransform(GetInternalObject().getFrameOffsetA());
}
umath::Transform pragma::physics::BtDoFSpringConstraint::GetFrameOffsetB() const
{
	return GetBtEnv().CreateTransform(GetInternalObject().getFrameOffsetB());
}
Vector3 pragma::physics::BtDoFSpringConstraint::GetAxis(pragma::Axis axisIndex) const
{
	return uvec::create(GetInternalObject().getAxis(umath::to_integral(axisIndex)));
}
double pragma::physics::BtDoFSpringConstraint::GetAngle(pragma::Axis axisIndex) const
{
	return GetInternalObject().getAngle(umath::to_integral(axisIndex));
}
double pragma::physics::BtDoFSpringConstraint::GetRelativePivotPosition(pragma::Axis axisIndex) const
{
	return GetInternalObject().getRelativePivotPosition(umath::to_integral(axisIndex));
}
void pragma::physics::BtDoFSpringConstraint::SetFrames(const umath::Transform &frameA,const umath::Transform &frameB)
{
	GetInternalObject().setFrames(GetBtEnv().CreateBtTransform(frameA),GetBtEnv().CreateBtTransform(frameB));
}
void pragma::physics::BtDoFSpringConstraint::SetLinearLowerLimit(const Vector3 &linearLower)
{
	GetInternalObject().setLinearLowerLimit(uvec::create_bt(linearLower) *BtEnvironment::WORLD_SCALE);
}
Vector3 pragma::physics::BtDoFSpringConstraint::GetLinearLowerLimit() const
{
	btVector3 r;
	GetInternalObject().getLinearLowerLimit(r);
	return uvec::create(r /BtEnvironment::WORLD_SCALE);
}
void pragma::physics::BtDoFSpringConstraint::SetLinearUpperLimit(const Vector3 &linearUpper)
{
	GetInternalObject().setLinearUpperLimit(uvec::create_bt(linearUpper) *BtEnvironment::WORLD_SCALE);
}
Vector3 pragma::physics::BtDoFSpringConstraint::GetLinearUpperLimit() const
{
	btVector3 r;
	GetInternalObject().getLinearUpperLimit(r);
	return uvec::create(r /BtEnvironment::WORLD_SCALE);
}
void pragma::physics::BtDoFSpringConstraint::SetAngularLowerLimit(const Vector3 &angularLower)
{
	GetInternalObject().setAngularLowerLimit(uvec::create_bt(angularLower));
}
void pragma::physics::BtDoFSpringConstraint::SetAngularLowerLimitReversed(const Vector3 &angularLower)
{
	GetInternalObject().setAngularLowerLimitReversed(uvec::create_bt(angularLower));
}
Vector3 pragma::physics::BtDoFSpringConstraint::GetAngularLowerLimit() const
{
	btVector3 r;
	GetInternalObject().getAngularLowerLimit(r);
	return uvec::create(r);
}
Vector3 pragma::physics::BtDoFSpringConstraint::GetAngularLowerLimitReversed() const
{
	btVector3 r;
	GetInternalObject().getAngularLowerLimitReversed(r);
	return uvec::create(r);
}
void pragma::physics::BtDoFSpringConstraint::SetAngularUpperLimit(const Vector3 &angularUpper)
{
	GetInternalObject().setAngularUpperLimit(uvec::create_bt(angularUpper));
}
void pragma::physics::BtDoFSpringConstraint::SetAngularUpperLimitReversed(const Vector3 &angularUpper)
{
	GetInternalObject().setAngularUpperLimitReversed(uvec::create_bt(angularUpper));
}
Vector3 pragma::physics::BtDoFSpringConstraint::GetAngularUpperLimit() const
{
	btVector3 r;
	GetInternalObject().getAngularUpperLimit(r);
	return uvec::create(r);
}
Vector3 pragma::physics::BtDoFSpringConstraint::GetAngularUpperLimitReversed() const
{
	btVector3 r;
	GetInternalObject().getAngularUpperLimitReversed(r);
	return uvec::create(r);
}
void pragma::physics::BtDoFSpringConstraint::SetLimit(AxisType type,pragma::Axis axis,double lo,double hi)
{
	if(type == AxisType::Linear)
	{
		lo *= BtEnvironment::WORLD_SCALE;
		hi *= BtEnvironment::WORLD_SCALE;
	}
	GetInternalObject().setLimit(get_axis_index(type,axis),lo,hi);
}
void pragma::physics::BtDoFSpringConstraint::SetLimitReversed(AxisType type,pragma::Axis axis,double lo,double hi)
{
	if(type == AxisType::Linear)
	{
		lo *= BtEnvironment::WORLD_SCALE;
		hi *= BtEnvironment::WORLD_SCALE;
	}
	GetInternalObject().setLimitReversed(get_axis_index(type,axis),lo,hi);
}
bool pragma::physics::BtDoFSpringConstraint::IsLimited(AxisType type,pragma::Axis axis) const
{
	return GetInternalObject().isLimited(get_axis_index(type,axis));
}
void pragma::physics::BtDoFSpringConstraint::SetRotationOrder(pragma::RotationOrder order)
{
	GetInternalObject().setRotationOrder(get_spring_rotation_order(order));
}
pragma::RotationOrder pragma::physics::BtDoFSpringConstraint::GetRotationOrder() const
{
	return get_rotation_order(GetInternalObject().getRotationOrder());
}
void pragma::physics::BtDoFSpringConstraint::SetAxis(const Vector3 &axis1,const Vector3 &axis2)
{
	GetInternalObject().setAxis(uvec::create_bt(axis1),uvec::create_bt(axis2));
}
void pragma::physics::BtDoFSpringConstraint::SetBounce(AxisType type,pragma::Axis axis,double bounce)
{
	GetInternalObject().setBounce(get_axis_index(type,axis),bounce);
}
void pragma::physics::BtDoFSpringConstraint::EnableMotor(AxisType type,pragma::Axis axis,bool onOff)
{
	GetInternalObject().enableMotor(get_axis_index(type,axis),onOff);
}
void pragma::physics::BtDoFSpringConstraint::SetServo(AxisType type,pragma::Axis axis,bool onOff)
{
	GetInternalObject().setServo(get_axis_index(type,axis),onOff);
}
void pragma::physics::BtDoFSpringConstraint::SetTargetVelocity(AxisType type,pragma::Axis axis,double velocity)
{
	GetInternalObject().setTargetVelocity(get_axis_index(type,axis),velocity);
}
void pragma::physics::BtDoFSpringConstraint::SetServoTarget(AxisType type,pragma::Axis axis,double target)
{
	GetInternalObject().setServoTarget(get_axis_index(type,axis),target);
}
void pragma::physics::BtDoFSpringConstraint::SetMaxMotorForce(AxisType type,pragma::Axis axis,double force)
{
	GetInternalObject().setMaxMotorForce(get_axis_index(type,axis),force);
}
void pragma::physics::BtDoFSpringConstraint::EnableSpring(AxisType type,pragma::Axis axis,bool onOff)
{
	GetInternalObject().enableSpring(get_axis_index(type,axis),onOff);
}
void pragma::physics::BtDoFSpringConstraint::SetStiffness(AxisType type,pragma::Axis axis,double stiffness,bool limitIfNeeded)
{
	GetInternalObject().setStiffness(get_axis_index(type,axis),stiffness,limitIfNeeded);
}
void pragma::physics::BtDoFSpringConstraint::SetDamping(AxisType type,pragma::Axis axis,double damping,bool limitIfNeeded)
{
	GetInternalObject().setDamping(get_axis_index(type,axis),damping,limitIfNeeded);
}
void pragma::physics::BtDoFSpringConstraint::SetEquilibriumPoint()
{
	GetInternalObject().setEquilibriumPoint();
}
void pragma::physics::BtDoFSpringConstraint::SetEquilibriumPoint(AxisType type,pragma::Axis axis)
{
	GetInternalObject().setEquilibriumPoint(get_axis_index(type,axis));
}
void pragma::physics::BtDoFSpringConstraint::SetEquilibriumPoint(AxisType type,pragma::Axis axis,double val)
{
	GetInternalObject().setEquilibriumPoint(get_axis_index(type,axis),val);
}
void pragma::physics::BtDoFSpringConstraint::SetERP(AxisType type,pragma::Axis axis,double value)
{
	GetInternalObject().setParam(BT_CONSTRAINT_ERP,value,get_axis_index(type,axis));
}
double pragma::physics::BtDoFSpringConstraint::GetERP(AxisType type,pragma::Axis axis) const
{
	return GetInternalObject().getParam(BT_CONSTRAINT_ERP,get_axis_index(type,axis));
}
void pragma::physics::BtDoFSpringConstraint::SetStopERP(AxisType type,pragma::Axis axis,double value)
{
	GetInternalObject().setParam(BT_CONSTRAINT_STOP_ERP,value,get_axis_index(type,axis));
}
double pragma::physics::BtDoFSpringConstraint::GetStopERP(AxisType type,pragma::Axis axis) const
{
	return GetInternalObject().getParam(BT_CONSTRAINT_STOP_ERP,get_axis_index(type,axis));
}
void pragma::physics::BtDoFSpringConstraint::SetCFM(AxisType type,pragma::Axis axis,double value)
{
	GetInternalObject().setParam(BT_CONSTRAINT_CFM,value,get_axis_index(type,axis));
}
double pragma::physics::BtDoFSpringConstraint::GetCFM(AxisType type,pragma::Axis axis) const
{
	return GetInternalObject().getParam(BT_CONSTRAINT_CFM,get_axis_index(type,axis));
}
void pragma::physics::BtDoFSpringConstraint::SetStopCFM(AxisType type,pragma::Axis axis,double value)
{
	GetInternalObject().setParam(BT_CONSTRAINT_STOP_CFM,value,get_axis_index(type,axis));
}
double pragma::physics::BtDoFSpringConstraint::GetStopCFM(AxisType type,pragma::Axis axis) const
{
	return GetInternalObject().getParam(BT_CONSTRAINT_STOP_CFM,get_axis_index(type,axis));
}
bool pragma::physics::BtDoFSpringConstraint::MatrixToEulerXYZ(const btMatrix3x3& mat,btVector3& xyz) const
{
	return GetInternalObject().matrixToEulerXYZ(mat,xyz);
}
bool pragma::physics::BtDoFSpringConstraint::MatrixToEulerXZY(const btMatrix3x3& mat,btVector3& xyz) const
{
	return GetInternalObject().matrixToEulerXZY(mat,xyz);
}
bool pragma::physics::BtDoFSpringConstraint::MatrixToEulerYXZ(const btMatrix3x3& mat,btVector3& xyz) const
{
	return GetInternalObject().matrixToEulerYXZ(mat,xyz);
}
bool pragma::physics::BtDoFSpringConstraint::MatrixToEulerYZX(const btMatrix3x3& mat,btVector3& xyz) const
{
	return GetInternalObject().matrixToEulerYZX(mat,xyz);
}
bool pragma::physics::BtDoFSpringConstraint::MatrixToEulerZXY(const btMatrix3x3& mat,btVector3& xyz) const
{
	return GetInternalObject().matrixToEulerZXY(mat,xyz);
}
bool pragma::physics::BtDoFSpringConstraint::MatrixToEulerZYX(const btMatrix3x3& mat,btVector3& xyz) const
{
	return GetInternalObject().matrixToEulerZYX(mat,xyz);
}
#pragma optimize("",on)
