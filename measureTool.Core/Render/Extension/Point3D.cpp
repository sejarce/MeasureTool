#include "Point3D.h"

#include "Render/RenderGroupID.h"
#include "Render/QueryMask.h"
#include "Render/PrefabResourceMgr.h"

#include "Ogre.h"

static const auto sDefaultRadius = 0.01f;

Point3D::Point3D(const Ogre::IdType& id, Ogre::ObjectMemoryManager *objectMemoryManager)
	:MovableObject(id, objectMemoryManager, ERG_Point)
{
	auto mesh = PrefabResourceMgr::GetInstance().GetMesh(PrefabResourceMgr::EPM_UnitSphere);
	mesh->load();

	mesh->getSubMesh(0)->_getRenderOperation(RO_);

	AABB_ = mesh->getBounds();

	setQueryFlags(EQF_Point);

	Material_ = PrefabResourceMgr::GetInstance().GetMateiral(PrefabResourceMgr::EPMAT_Point);

	setCustomParameter(0, Ogre::Vector4(Ogre::Vector3::UNIT_SCALE));

	LocalMat_ = Ogre::Matrix4::IDENTITY;

	Pnt_ = Ogre::Vector3(-0.f, 0.f, 0.f);

	SetColor(Ogre::ColourValue::Red);
	SetRadius(sDefaultRadius);
}

Point3D::~Point3D()
{
	
}

const Ogre::String& Point3D::getMovableType(void) const
{
	return Point3DFactory::GetFactoryName();
}

const Ogre::LightList& Point3D::getLights(void) const
{
	return queryLights();
}

void Point3D::getRenderOperation(Ogre::RenderOperation& op)
{
	op = RO_;
}

void Point3D::getWorldTransforms(Ogre::Matrix4* xform) const
{	
	*xform = getParentNode()->_getFullTransform() * LocalMat_;
}

void Point3D::_updateRenderQueue(Ogre::RenderQueue* queue, Ogre::Camera *camera, const Ogre::Camera *lodCamera)
{
	queue->addRenderable(this, mRenderQueueID, mRenderQueuePriority);
}

void Point3D::visitRenderables(Renderable::Visitor* visitor, bool debugRenderables /*= false*/)
{
	visitor->visit(this, 0, false);
}

const Ogre::MaterialPtr& Point3D::getMaterial(void) const
{
	return Material_;
}

Ogre::Real Point3D::getSquaredViewDepth(const Ogre::Camera* cam) const
{
	auto diff = (getParentNode()->_getFullTransform() * LocalMat_).getTrans() - cam->getDerivedPosition();
	return diff.squaredLength();
}

const Ogre::UserObjectBindings& Point3D::GetUserObjectBindings() const
{
	return Ogre::MovableObject::getUserObjectBindings();
}

Ogre::UserObjectBindings& Point3D::GetUserObjectBindings()
{
	return Ogre::MovableObject::getUserObjectBindings();
}

void Point3D::SetSmgr(Ogre::SceneManager* smgr)
{
	Smgr_ = smgr;
}

Ogre::SceneManager* Point3D::GetSmgr() const
{
	return Smgr_;
}

void Point3D::Destory()
{
	detachFromParent();

	Smgr_->destroyMovableObject(this);
}

void Point3D::SetColor(const Ogre::ColourValue& val)
{
	Ogre::Vector4 clr;
	clr.x = val.r;
	clr.y = val.g;
	clr.z = val.b;
	clr.w = val.a;
	setCustomParameter(0, clr);
}

Ogre::ColourValue Point3D::GetColor() const
{
	auto val = getCustomParameter(0);
	return Ogre::ColourValue(val.x, val.y, val.z, val.w);
}

void Point3D::SetRadius(float val)
{
	Radius_ = val;

	_UpdateTransform();
}

float Point3D::GetRadius() const
{
	return Radius_;
}

void Point3D::SetPoint(const Ogre::Vector3& pos)
{
	Pnt_ = pos;

	_UpdateTransform();
}

const Ogre::Vector3& Point3D::GetPoint() const
{
	return Pnt_;
}

void Point3D::_UpdateTransform()
{
	LocalMat_.makeTransform(Pnt_,
							Ogre::Vector3(Radius_, Radius_, Radius_),
							Ogre::Quaternion::IDENTITY);


	Ogre::Aabb aabb(AABB_.getCenter(), AABB_.getHalfSize());
	aabb.transformAffine(LocalMat_);
	mObjectData.mLocalAabb->setFromAabb(aabb, mObjectData.mIndex);
	mObjectData.mLocalRadius[mObjectData.mIndex] = aabb.getRadius();
}