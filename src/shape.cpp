#include "shape.hpp"
#include "environment.hpp"
#include <pragma/physics/collisionmesh.h>
#include <pragma/math/surfacematerial.h>
#include <pragma/physics/physedgeutility.h>
#include <BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.h>

//////////////////////////////////

pragma::physics::BtShape::BtShape(IEnvironment &env,const std::shared_ptr<btCollisionShape> &shape)
	: IShape{env},m_shape{shape},m_externalShape{nullptr}
{}
pragma::physics::BtShape::BtShape(IEnvironment &env,btCollisionShape *shape,bool bOwns)
	: IShape{env},m_shape(nullptr),m_externalShape(nullptr)
{
	if(bOwns == true)
		m_shape = std::shared_ptr<btCollisionShape>(shape);
	else
		m_externalShape = shape;
}

void pragma::physics::BtShape::GetBoundingSphere(Vector3 &outCenter,float &outRadius) const
{
	auto &shape = GetBtShape();
	btVector3 origin;
	btScalar radius;
	shape.getBoundingSphere(origin,radius);
	outCenter = Vector3{origin.x() /BtEnvironment::WORLD_SCALE,origin.y() /BtEnvironment::WORLD_SCALE,origin.z() /BtEnvironment::WORLD_SCALE};
	outRadius = radius /BtEnvironment::WORLD_SCALE;
}

void pragma::physics::BtShape::SetTrigger(bool bTrigger) {m_bTrigger = bTrigger;}
bool pragma::physics::BtShape::IsTrigger() const {return m_bTrigger;}

void pragma::physics::BtShape::GetAABB(Vector3 &min,Vector3 &max) const
{
	if(m_shape == nullptr)
	{
		min = {};
		max = {};
		return;
	}
	btTransform t {};
	t.setIdentity();
	btVector3 btMin,btMax;
	m_shape->getAabb(t,btMin,btMax);
	min = uvec::create(btMin /BtEnvironment::WORLD_SCALE);
	max = uvec::create(btMax /BtEnvironment::WORLD_SCALE);
}

void pragma::physics::BtShape::CalculateLocalInertia(float mass,Vector3 *localInertia) const
{
	if(m_shape == nullptr)
		return;
	btVector3 btInertia;
	m_shape->calculateLocalInertia(mass,btInertia);
	localInertia->x = static_cast<float>(btInertia.x() /BtEnvironment::WORLD_SCALE);
	localInertia->y = static_cast<float>(btInertia.y() /BtEnvironment::WORLD_SCALE);
	localInertia->z = static_cast<float>(btInertia.z() /BtEnvironment::WORLD_SCALE);
}

btCollisionShape &pragma::physics::BtShape::GetBtShape() {return (m_externalShape != nullptr) ? *m_externalShape : *m_shape;}
const btCollisionShape &pragma::physics::BtShape::GetBtShape() const {return const_cast<BtShape*>(this)->GetBtShape();}

//////////////////////////////////

pragma::physics::BtConvexShape::BtConvexShape(IEnvironment &env,const std::shared_ptr<btConvexShape> &shape)
	: IConvexShape{env},BtShape{env,shape},IShape{env}
{}
btConvexShape &pragma::physics::BtConvexShape::GetBtConvexShape() {return static_cast<btConvexShape&>(GetBtShape());}

void pragma::physics::BtConvexShape::SetLocalScaling(const Vector3 &scale)
{
	GetBtConvexShape().setLocalScaling(btVector3(scale.x,scale.y,scale.z));
}

//////////////////////////////////

pragma::physics::BtConvexHullShape::BtConvexHullShape(IEnvironment &env,const std::shared_ptr<btConvexHullShape> &shape)
	: IShape{env},IConvexShape{env},IConvexHullShape{env},BtConvexShape{env,shape},m_surfaceMaterial(0)
{}
btConvexHullShape &pragma::physics::BtConvexHullShape::GetBtConvexHullShape() {return static_cast<btConvexHullShape&>(GetBtShape());}
void pragma::physics::BtConvexHullShape::SetSurfaceMaterial(int id) {m_surfaceMaterial = id;}
int pragma::physics::BtConvexHullShape::GetSurfaceMaterial() const {return m_surfaceMaterial;}

void pragma::physics::BtConvexHullShape::AddPoint(const Vector3 &point)
{
	GetBtConvexHullShape().addPoint(btVector3(point.x,point.y,point.z) *BtEnvironment::WORLD_SCALE);
}

//////////////////////////////////

pragma::physics::BtHeightfield::BtHeightfield(IEnvironment &env,const std::shared_ptr<btHeightfieldTerrainShape> &shape,uint32_t width,uint32_t length,btScalar maxHeight,uint8_t upAxis)
	: IHeightfield{env,width,length,maxHeight,upAxis},BtShape{env,shape},IShape{env}
{}
uint32_t pragma::physics::BtHeightfield::GetWidth() const {return m_width;}
uint32_t pragma::physics::BtHeightfield::GetLength() const {return m_length;}
float pragma::physics::BtHeightfield::GetMaxHeight() const {return m_maxHeight;}
uint8_t pragma::physics::BtHeightfield::GetUpAxis() const {return m_upAxis;}
float pragma::physics::BtHeightfield::GetHeight(uint32_t x,uint32_t y) const
{
	auto idx = y *GetWidth() +x;
	if(idx >= m_heightFieldData.size())
		return 0.f;
	return m_heightFieldData.at(idx);
}
void pragma::physics::BtHeightfield::SetHeight(uint32_t x,uint32_t y,float height)
{
	auto idx = y *GetWidth() +x;
	if(idx >= m_heightFieldData.size())
		return;
	m_heightFieldData.at(idx) = height *BtEnvironment::WORLD_SCALE;
}

//////////////////////////////////

pragma::physics::BtTriangleShape::BtTriangleShape(IEnvironment &env,const std::shared_ptr<btBvhTriangleMeshShape> &shape)
	: ITriangleShape{env},BtShape{env,shape},IShape{env}
{}
void pragma::physics::BtTriangleShape::AddTriangle(const Vector3 &a,const Vector3 &b,const Vector3 &c,const SurfaceMaterial *mat)
{
	assert(!m_bBuilt); // If already built, we'd have to remove our btTriangleIndexVertexMaterialArray, due to its internal structure (It points directly to our vector data). 
	m_vertices.push_back(a);
	m_triangles.push_back(static_cast<int>(m_vertices.size() -1));

	m_vertices.push_back(b);
	m_triangles.push_back(static_cast<int>(m_vertices.size() -1));

	m_vertices.push_back(c);
	m_triangles.push_back(static_cast<int>(m_vertices.size() -1));

	if(mat != nullptr)
		m_faceMaterials.push_back(static_cast<int>(mat->GetIndex()));
	else
		m_faceMaterials.push_back(0);
	/*m_triangleMesh->addTriangle(
	btVector3(a.x,a.y,a.z),
	btVector3(b.x,b.y,b.z),
	btVector3(c.x,c.y,c.z),
	false
	);*/
}

void pragma::physics::BtTriangleShape::ReserveTriangles(std::size_t count)
{
	m_vertices.reserve(count *3);
	m_triangles.reserve(count *3);
	m_faceMaterials.reserve(count);
}
btTriangleIndexVertexArray *pragma::physics::BtTriangleShape::GetBtIndexVertexArray() {return m_triangleArray.get();}

void pragma::physics::BtTriangleShape::Build(const std::vector<SurfaceMaterial> *materials)
{
	m_shape = nullptr;
	if(m_triangles.empty() || m_vertices.empty())
		return;
	std::vector<btVector3> btVerts {};
	btVerts.reserve(m_vertices.size());
	for(auto &v : m_vertices)
		btVerts.push_back(btVector3(v.x,v.y,v.z));
	btBvhTriangleMeshShape *shape = nullptr;
	if(materials == nullptr)
	{
		m_triangleArray = std::make_unique<btTriangleIndexVertexArray>(
			static_cast<int>(m_triangles.size() /3),&m_triangles[0],static_cast<int>(sizeof(int) *3),
			static_cast<int>(btVerts.size()),reinterpret_cast<btScalar*>(btVerts.data()),static_cast<int>(sizeof(btVector3))
		);
		shape = new btBvhTriangleMeshShape(m_triangleArray.get(),false);
	}
	else
	{
		m_triangleArray = std::make_unique<btTriangleIndexVertexMaterialArray>(
			static_cast<int>(m_triangles.size() /3),&m_triangles[0],static_cast<int>(sizeof(int) *3),
			static_cast<int>(btVerts.size()),reinterpret_cast<btScalar*>(btVerts.data()),static_cast<int>(sizeof(btVector3)),
			static_cast<int>(materials->size()),(unsigned char*)(&(*materials)[0]),static_cast<int>(sizeof(SurfaceMaterial)),
			&m_faceMaterials[0],static_cast<int>(sizeof(int))
		);
		shape = new btMultimaterialTriangleMeshShape(m_triangleArray.get(),false);
	}
	m_bBuilt = true;
	m_shape = std::shared_ptr<btCollisionShape>(shape);
	m_infoMap = std::make_unique<btTriangleInfoMap>();
	GenerateInternalEdgeInfo();
}

void pragma::physics::BtTriangleShape::GenerateInternalEdgeInfo()
{
	static_cast<btBvhTriangleMeshShape*>(m_shape.get())->setTriangleInfoMap(nullptr);
	btGenerateInternalEdgeInfo(static_cast<btBvhTriangleMeshShape*>(m_shape.get()),m_infoMap.get());
}
void pragma::physics::BtTriangleShape::CalculateLocalInertia(float,Vector3 *localInertia) const
{
	*localInertia = Vector3(0.f,0.f,0.f);
}

//////////////////////////////////

pragma::physics::BtCompoundShape::BtCompoundShape(IEnvironment &env,const std::shared_ptr<btCompoundShape> &btShape,const std::vector<IShape*> &shapes)
	: BtShape{env,btShape},IShape{env},ICompoundShape{env,shapes}
{}
btCompoundShape &pragma::physics::BtCompoundShape::GetBtCompoundShape() {return static_cast<btCompoundShape&>(GetBtShape());}
void pragma::physics::BtCompoundShape::AddShape(pragma::physics::IShape &shape,const Vector3 &origin,const Quat &rot)
{
	btTransform t;
	t.setIdentity();
	t.setOrigin(btVector3(origin.x,origin.y,origin.z) *BtEnvironment::WORLD_SCALE);
	t.setRotation(uquat::create_bt(rot));
	GetBtCompoundShape().addChildShape(t,&dynamic_cast<BtShape&>(shape).GetBtShape());
	m_shapes.push_back(std::static_pointer_cast<IShape>(shape.shared_from_this()));
}
