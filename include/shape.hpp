#ifndef __PR_PX_SHAPE_HPP__
#define __PR_PX_SHAPE_HPP__

#include <pragma/physics/shape.hpp>
#include "common.hpp"

namespace pragma::physics
{
	class BtEnvironment;
	class btShape;
	class BtShape
		: virtual public pragma::physics::IShape
	{
	public:
		friend IEnvironment;
		BtShape(IEnvironment &env,const std::shared_ptr<btCollisionShape> &shape);
		virtual void CalculateLocalInertia(float mass,Vector3 *localInertia) const override;
		virtual void GetAABB(Vector3 &min,Vector3 &max) const override;
		virtual void GetBoundingSphere(Vector3 &outCenter,float &outRadius) const override;

		virtual void SetTrigger(bool bTrigger) override;
		virtual bool IsTrigger() const override;

		virtual void SetLocalPose(const Transform &t) override;
		virtual const Transform &GetLocalPose() const override;

		btCollisionShape &GetBtShape();
		const btCollisionShape &GetBtShape() const;
	protected:
		std::shared_ptr<btCollisionShape> m_shape;
	private:
		BtShape(IEnvironment &env,btCollisionShape *shape,bool bOwns=true);
		btCollisionShape *m_externalShape;
		Transform m_localPose = {};
		bool m_bTrigger = false;
	};
	class BtConvexShape
		: virtual public pragma::physics::IConvexShape,
		public BtShape
	{
	public:
		friend IEnvironment;
		btConvexShape &GetBtConvexShape();

		virtual void SetLocalScaling(const Vector3 &scale) override;
	protected:
		BtConvexShape(IEnvironment &env,const std::shared_ptr<btConvexShape> &shape);
	};
	class BtConvexHullShape
		: virtual public pragma::physics::IConvexHullShape,
		public BtConvexShape
	{
	public:
		friend IEnvironment;
		btConvexHullShape &GetBtConvexHullShape();

		virtual void AddPoint(const Vector3 &point) override;
		virtual void AddTriangle(uint32_t idx0,uint32_t idx1,uint32_t idx2) override;
		virtual void ReservePoints(uint32_t numPoints) override;
		virtual void ReserveTriangles(uint32_t numTris) override;
		virtual void Build() override;
	private:
		BtConvexHullShape(IEnvironment &env,const std::shared_ptr<btConvexHullShape> &shape);
	};

	class BtCompoundShape
		: virtual public pragma::physics::ICompoundShape,
		public BtShape
	{
	public:
		friend IEnvironment;
		btCompoundShape &GetBtCompoundShape();
		virtual void AddShape(pragma::physics::IShape &shape) override;
	protected:
		BtCompoundShape(IEnvironment &env,const std::shared_ptr<btCompoundShape> &btShape,const std::vector<IShape*> &shapes={});
	};

	class BtHeightfield
		: virtual public pragma::physics::IHeightfield,
		public BtShape
	{
	public:
		friend IEnvironment;
		virtual float GetHeight(uint32_t x,uint32_t y) const override;
		virtual void SetHeight(uint32_t x,uint32_t y,float height) override;
		virtual uint32_t GetWidth() const override;
		virtual uint32_t GetLength() const override;
		virtual float GetMaxHeight() const override;
		virtual uint8_t GetUpAxis() const override;
	protected:
		BtHeightfield(IEnvironment &env,const std::shared_ptr<btHeightfieldTerrainShape> &shape,uint32_t width,uint32_t length,btScalar maxHeight,uint8_t upAxis);
	};

	class BtTriangleShape
		: virtual public pragma::physics::ITriangleShape,
		public BtShape
	{
	public:
		friend IEnvironment;
		btTriangleIndexVertexArray *GetBtIndexVertexArray();

		virtual void Build(const std::vector<SurfaceMaterial> *materials=nullptr) override;
		void GenerateInternalEdgeInfo();

		virtual void AddTriangle(const Vector3 &a,const Vector3 &b,const Vector3 &c,const SurfaceMaterial *mat=nullptr) override;

		virtual void CalculateLocalInertia(float mass,Vector3 *localInertia) const override;
	private:
		BtTriangleShape(IEnvironment &env,const std::shared_ptr<btBvhTriangleMeshShape> &shape=nullptr);
		std::unique_ptr<btTriangleIndexVertexArray> m_triangleArray;
		std::unique_ptr<btTriangleInfoMap> m_infoMap = nullptr;
	};
};

#endif
