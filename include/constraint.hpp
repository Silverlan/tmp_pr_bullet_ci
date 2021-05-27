/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef __PR_BT_CONSTRAINT_HPP__
#define __PR_BT_CONSTRAINT_HPP__

#include "common.hpp"
#include <pragma/physics/constraint.hpp>

class btTypedConstraint;
class btFixedConstraint;
class btPoint2PointConstraint;
class btHingeConstraint;
class btSliderConstraint;
class btConeTwistConstraint;
class btGeneric6DofConstraint;
class btGeneric6DofSpring2Constraint;
class btRotationalLimitMotor2;
class btTranslationalLimitMotor2;
namespace pragma::physics
{
	class BtEnvironment;
	class BtFixedConstraint;
	class BtBallSocketConstraint;
	class BtHingeConstraint;
	class BtSliderConstraint;
	class BtConeTwistConstraint;
	class BtDoFConstraint;
	class BtDoFSpringConstraint;
	class BtConstraint
		: virtual public IConstraint
	{
	public:
		friend IEnvironment;
		btTypedConstraint &GetInternalObject() const;
		btTypedConstraint &GetBtConstraint() const;

		virtual void SetEnabled(bool b) override;
		virtual bool IsEnabled() const override;
		virtual bool IsBroken() const override;
		virtual void Break() override;

		virtual float GetBreakForce() const override;
		virtual void SetBreakForce(float threshold) override;
		virtual float GetBreakTorque() const override;
		virtual void SetBreakTorque(float torque) override;

		virtual void SetSoftness(float softness) override;
		virtual void SetDamping(float damping) override;
		virtual void SetRestitution(float restitution) override;

		virtual float GetSoftness() const override;
		virtual float GetDamping() const override;
		virtual float GetRestitution() const override;

		virtual BtFixedConstraint *GetBtFixedConstraint();
		const BtFixedConstraint *GetBtFixedConstraint() const;
		virtual BtBallSocketConstraint *GetBtBallSocketConstraint();
		const BtBallSocketConstraint *GetBtBallSocketConstraint() const;
		virtual BtHingeConstraint *GetBtHingeConstraint();
		const BtHingeConstraint *GetBtHingeConstraint() const;
		virtual BtSliderConstraint *GetBtSliderConstraint();
		const BtSliderConstraint *GetBtSliderConstraint() const;
		virtual BtConeTwistConstraint *GetBtConeTwistConstraint();
		const BtConeTwistConstraint *GetBtConeTwistConstraint() const;
		virtual BtDoFConstraint *GetBtDoFConstraint();
		const BtDoFConstraint *GetBtDoFConstraint() const;
		virtual BtDoFSpringConstraint *GetBtDoFSpringConstraint();
		const BtDoFSpringConstraint *GetBtDoFSpringConstraint() const;

		virtual void Initialize() override;
		virtual pragma::physics::IRigidBody *GetSourceActor() override;
		virtual pragma::physics::IRigidBody *GetTargetActor() override;
	protected:
		BtConstraint(IEnvironment &env,std::unique_ptr<btTypedConstraint> c);
		BtEnvironment &GetBtEnv() const;
		virtual void DoSetCollisionsEnabled(Bool b) override;
		virtual void RemoveWorldObject() override;
		virtual void DoAddWorldObject() override;
		std::unique_ptr<btTypedConstraint> m_constraint = nullptr;
	};

	class BtFixedConstraint
		: public IFixedConstraint,
		public BtConstraint
	{
	public:
		friend IEnvironment;
		btFixedConstraint &GetInternalObject() const;
		virtual BtFixedConstraint *GetBtFixedConstraint() override;
	protected:
		BtFixedConstraint(IEnvironment &env,std::unique_ptr<btFixedConstraint> constraint);
	};

	class BtBallSocketConstraint
		: public IBallSocketConstraint,
		public BtConstraint
	{
	public:
		friend IEnvironment;
		btPoint2PointConstraint &GetInternalObject() const;
		virtual BtBallSocketConstraint *GetBtBallSocketConstraint() override;
	protected:
		BtBallSocketConstraint(IEnvironment &env,std::unique_ptr<btPoint2PointConstraint> constraint);
	};

	class BtHingeConstraint
		: public IHingeConstraint,
		public BtConstraint
	{
	public:
		friend IEnvironment;
		btHingeConstraint &GetInternalObject() const;
		virtual BtHingeConstraint *GetBtHingeConstraint() override;
		virtual void SetLimit(umath::Radian lowerLimit,umath::Radian upperLimit) override;
		virtual std::pair<umath::Radian,umath::Radian> GetLimit() const override;
		virtual void DisableLimit() override;
	protected:
		BtHingeConstraint(IEnvironment &env,std::unique_ptr<btHingeConstraint> constraint);
		void SetLimit(float low,float high,float softness=0.9f,float biasFactor=0.3f,float relaxationFactor=1.f);
	};

	class BtSliderConstraint
		: public ISliderConstraint,
		public BtConstraint
	{
	public:
		friend IEnvironment;
		btSliderConstraint &GetInternalObject() const;
		virtual BtSliderConstraint *GetBtSliderConstraint() override;
		virtual void SetLimit(float lowerLimit,float upperLimit) override;
		virtual void DisableLimit() override;
		virtual std::pair<float,float> GetLimit() const override;
	protected:
		BtSliderConstraint(IEnvironment &env,std::unique_ptr<btSliderConstraint> constraint);
	};

	class BtConeTwistConstraint
		: public IConeTwistConstraint,
		public BtConstraint
	{
	public:
		friend IEnvironment;
		btConeTwistConstraint &GetInternalObject() const;
		virtual void SetLimit(const Vector3 &lowerLimits,const Vector3 &upperLimits) override;
		virtual void SetLimit(float swingSpan1,float swingSpan2,float twistSpan) override;
		virtual void GetLimit(float &outSwingSpan1,float &outSwingSpan2,float &outTwistSpan) override;
		virtual BtConeTwistConstraint *GetBtConeTwistConstraint() override;
	protected:
		BtConeTwistConstraint(IEnvironment &env,std::unique_ptr<btConeTwistConstraint> constraint);
	};

	class BtDoFConstraint
		: public IDoFConstraint,
		public BtConstraint
	{
	public:
		friend IEnvironment;
		btGeneric6DofConstraint &GetInternalObject() const;
		virtual BtDoFConstraint *GetBtDoFConstraint() override;
		virtual void SetLinearLimit(const Vector3 &lower,const Vector3 &upper) override;
		virtual void SetLinearLimit(const Vector3 &lim) override;
		virtual void SetLinearLowerLimit(const Vector3 &lim) override;
		virtual void SetLinearUpperLimit(const Vector3 &lim) override;
		virtual void SetAngularLimit(const EulerAngles &lower,const EulerAngles &upper) override;
		virtual void SetAngularLimit(const EulerAngles &lim) override;
		virtual void SetAngularLowerLimit(const EulerAngles &lim) override;
		virtual void SetAngularUpperLimit(const EulerAngles &lim) override;

		virtual Vector3 GetLinearLowerLimit() const override;
		virtual Vector3 GetlinearUpperLimit() const override;
		virtual EulerAngles GetAngularLowerLimit() const override;
		virtual EulerAngles GetAngularUpperLimit() const override;

		virtual Vector3 GetAngularTargetVelocity() const override;
		virtual void SetAngularTargetVelocity(const Vector3 &vel) const override;
		virtual Vector3 GetAngularMaxMotorForce() const override;
		virtual void SetAngularMaxMotorForce(const Vector3 &force) override;
		virtual Vector3 GetAngularMaxLimitForce() const override;
		virtual void SetAngularMaxLimitForce(const Vector3 &force) override;
		virtual Vector3 GetAngularDamping() const override;
		virtual void SetAngularDamping(const Vector3 &damping) override;
		virtual Vector3 GetAngularLimitSoftness() const override;
		virtual void SetAngularLimitSoftness(const Vector3 &softness) override;
		virtual Vector3 GetAngularForceMixingFactor() const override;
		virtual void SetAngularForceMixingFactor(const Vector3 &factor) override;
		virtual Vector3 GetAngularLimitErrorTolerance() const override;
		virtual void SetAngularLimitErrorTolerance(const Vector3 &tolerance) override;
		virtual Vector3 GetAngularLimitForceMixingFactor() const override;
		virtual void SetAngularLimitForceMixingFactor(const Vector3 &factor) override;
		virtual Vector3 GetAngularRestitutionFactor() const override;
		virtual void SetAngularRestitutionFactor(const Vector3 &factor) override;
		virtual bool IsAngularMotorEnabled(uint8_t axis) const override;
		virtual void SetAngularMotorEnabled(uint8_t axis,bool bEnabled) override;
		virtual Vector3 GetCurrentAngularLimitError() const override;
		virtual Vector3 GetCurrentAngularPosition() const override;
		virtual Vector3i GetCurrentAngularLimit() const override;
		virtual Vector3 GetCurrentAngularAccumulatedImpulse() const override;

		virtual Vector3 GetLinearTargetVelocity() const override;
		virtual void SetLinearTargetVelocity(const Vector3 &vel) const override;
		virtual Vector3 GetLinearMaxMotorForce() const override;
		virtual void SetLinearMaxMotorForce(const Vector3 &force) override;
		virtual float GetLinearDamping() const override;
		virtual void SetLinearDamping(float damping) override;
		virtual float GetLinearLimitSoftness() const override;
		virtual void SetLinearLimitSoftness(float softness) override;
		virtual Vector3 GetLinearForceMixingFactor() const override;
		virtual void SetLinearForceMixingFactor(const Vector3 &factor) override;
		virtual Vector3 GetLinearLimitErrorTolerance() const override;
		virtual void SetLinearLimitErrorTolerance(const Vector3 &tolerance) override;
		virtual Vector3 GetLinearLimitForceMixingFactor() const override;
		virtual void SetLinearLimitForceMixingFactor(const Vector3 &factor) override;
		virtual float GetLinearRestitutionFactor() const override;
		virtual void SetLinearRestitutionFactor(float factor) override;
		virtual bool IsLinearMotorEnabled(uint8_t axis) const override;
		virtual void SetLinearMotorEnabled(uint8_t axis,bool bEnabled) override;
		virtual Vector3 GetCurrentLinearDifference() const override;
		virtual Vector3 GetCurrentLinearLimitError() const override;
		virtual Vector3i GetCurrentLinearLimit() const override;
		virtual Vector3 GetCurrentLinearAccumulatedImpulse() const override;
	protected:
		BtDoFConstraint(IEnvironment &env,std::unique_ptr<btGeneric6DofConstraint> constraint);
	};

	class BtDoFSpringConstraint
		: public IDoFSpringConstraint,
		public BtConstraint
	{
	public:
		friend IEnvironment;
		btGeneric6DofSpring2Constraint &GetInternalObject() const;
		virtual BtDoFSpringConstraint *GetBtDoFSpringConstraint() override;

		virtual void CalculateTransforms() override;
		virtual void CalculateTransforms(const umath::Transform &frameA,const umath::Transform &frameB) override;
		btRotationalLimitMotor2 *GetRotationalLimitMotor(pragma::Axis index) const;
		btTranslationalLimitMotor2 *GetTranslationalLimitMotor() const;
		virtual umath::Transform GetCalculatedTransformA() const override;
		virtual umath::Transform GetCalculatedTransformB() const override;
		virtual umath::Transform GetFrameOffsetA() const override;
		virtual umath::Transform GetFrameOffsetB() const override;
		virtual Vector3 GetAxis(pragma::Axis axisIndex) const override;
		virtual double GetAngle(pragma::Axis axisIndex) const override;
		virtual double GetRelativePivotPosition(pragma::Axis axisIndex) const override;
		virtual void SetFrames(const umath::Transform &frameA,const umath::Transform &frameB) override;
		virtual void SetLinearLowerLimit(const Vector3 &linearLower) override;
		virtual Vector3 GetLinearLowerLimit() const override;
		virtual void SetLinearUpperLimit(const Vector3 &linearUpper) override;
		virtual Vector3 GetLinearUpperLimit() const override;
		virtual void SetAngularLowerLimit(const Vector3 &angularLower) override;
		virtual void SetAngularLowerLimitReversed(const Vector3 &angularLower) override;
		virtual Vector3 GetAngularLowerLimit() const override;
		virtual Vector3 GetAngularLowerLimitReversed() const override;
		virtual void SetAngularUpperLimit(const Vector3 &angularUpper) override;
		virtual void SetAngularUpperLimitReversed(const Vector3 &angularUpper) override;
		virtual Vector3 GetAngularUpperLimit() const override;
		virtual Vector3 GetAngularUpperLimitReversed() const override;
		virtual void SetLimit(AxisType type,pragma::Axis axis,double lo,double hi) override;
		virtual void SetLimitReversed(AxisType type,pragma::Axis axis,double lo,double hi) override;
		virtual bool IsLimited(AxisType type,pragma::Axis axis) const override;
		virtual void SetRotationOrder(pragma::RotationOrder order) override;
		virtual pragma::RotationOrder GetRotationOrder() const override;
		virtual void SetAxis(const Vector3 &axis1,const Vector3 &axis2) override;
		virtual void SetBounce(AxisType type,pragma::Axis axis,double bounce) override;
		virtual void EnableMotor(AxisType type,pragma::Axis axis,bool onOff) override;
		virtual void SetServo(AxisType type,pragma::Axis axis,bool onOff) override;
		virtual void SetTargetVelocity(AxisType type,pragma::Axis axis,double velocity) override;
		virtual void SetServoTarget(AxisType type,pragma::Axis axis,double target) override;
		virtual void SetMaxMotorForce(AxisType type,pragma::Axis axis,double force) override;
		virtual void EnableSpring(AxisType type,pragma::Axis axis,bool onOff) override;
		virtual void SetStiffness(AxisType type,pragma::Axis axis,double stiffness,bool limitIfNeeded=true) override;
		virtual void SetDamping(AxisType type,pragma::Axis axis,double damping,bool limitIfNeeded=true) override;
		virtual void SetEquilibriumPoint() override;
		virtual void SetEquilibriumPoint(AxisType type,pragma::Axis axis) override;
		virtual void SetEquilibriumPoint(AxisType type,pragma::Axis axis,double val) override;

		virtual void SetERP(AxisType type,pragma::Axis axis,double value) override;
		virtual double GetERP(AxisType type,pragma::Axis axis) const override;
		virtual void SetStopERP(AxisType type,pragma::Axis axis,double value) override;
		virtual double GetStopERP(AxisType type,pragma::Axis axis) const override;
		virtual void SetCFM(AxisType type,pragma::Axis axis,double value) override;
		virtual double GetCFM(AxisType type,pragma::Axis axis) const override;
		virtual void SetStopCFM(AxisType type,pragma::Axis axis,double value) override;
		virtual double GetStopCFM(AxisType type,pragma::Axis axis) const override;

		bool MatrixToEulerXYZ(const btMatrix3x3& mat,btVector3& xyz) const;
		bool MatrixToEulerXZY(const btMatrix3x3& mat,btVector3& xyz) const;
		bool MatrixToEulerYXZ(const btMatrix3x3& mat,btVector3& xyz) const;
		bool MatrixToEulerYZX(const btMatrix3x3& mat,btVector3& xyz) const;
		bool MatrixToEulerZXY(const btMatrix3x3& mat,btVector3& xyz) const;
		bool MatrixToEulerZYX(const btMatrix3x3& mat,btVector3& xyz) const;
	protected:
		BtDoFSpringConstraint(IEnvironment &env,std::unique_ptr<btGeneric6DofSpring2Constraint> constraint);
	};
};

#endif
