#include "Line3D.h"

#include "Render/RenderGroupID.h"
#include "Render/QueryMask.h"
#include "Render/PrefabResourceMgr.h"

#include "Ogre.h"

static const auto sDefaultWidth = 0.002f;

Line3D::Line3D(const Ogre::IdType& id, Ogre::ObjectMemoryManager *objectMemoryManager)
	:MovableObject(id, objectMemoryManager, ERG_Line)
{
	auto mesh = PrefabResourceMgr::GetInstance().GetMesh(PrefabResourceMgr::EPM_UnitLine);
	mesh->load();

	mesh->getSubMesh(0)->_getRenderOperation(RO_);

	AABB_ = mesh->getBounds();

	setQueryFlags(EQF_Line);

	Material_ = PrefabResourceMgr::GetInstance().GetMateiral(PrefabResourceMgr::EPMAT_Curve);

	setCustomParameter(0, Ogre::Vector4(Ogre::Vector3::UNIT_SCALE));

	LocalMat_ = Ogre::Matrix4::IDENTITY;

	StartPnt_ = Ogre::Vector3(-.5f, 0.f, 0.f);
	EndPnt_ = Ogre::Vector3(.5f, 0.f, 0.f);

	SetColor(Ogre::ColourValue::White);
	SetLineWidth(sDefaultWidth);
}

Line3D::~Line3D()
{
	
}

const Ogre::String& Line3D::getMovableType(void) const
{
	return Line3DFactory::GetFactoryName();
}

const Ogre::LightList& Line3D::getLights(void) const
{
	return queryLights();
}

void Line3D::getRenderOperation(Ogre::RenderOperation& op)
{
	op = RO_;
}

void Line3D::getWorldTransforms(Ogre::Matrix4* xform) const
{	
	*xform = getParentNode()->_getFullTransform() * LocalMat_;
}

void Line3D::_updateRenderQueue(Ogre::RenderQueue* queue, Ogre::Camera *camera, const Ogre::Camera *lodCamera)
{
	queue->addRenderable(this, mRenderQueueID, mRenderQueuePriority);
}

void Line3D::visitRenderables(Renderable::Visitor* visitor, bool debugRenderables /*= false*/)
{
	visitor->visit(this, 0, false);
}

const Ogre::MaterialPtr& Line3D::getMaterial(void) const
{
	return Material_;
}

Ogre::Real Line3D::getSquaredViewDepth(const Ogre::Camera* cam) const
{
	auto diff = (getParentNode()->_getFullTransform() * LocalMat_).getTrans() - cam->getDerivedPosition();
	return diff.squaredLength();
}

const Ogre::UserObjectBindings& Line3D::GetUserObjectBindings() const
{
	return Ogre::MovableObject::getUserObjectBindings();
}

Ogre::UserObjectBindings& Line3D::GetUserObjectBindings()
{
	return Ogre::MovableObject::getUserObjectBindings();
}

void Line3D::SetSmgr(Ogre::SceneManager* smgr)
{
	Smgr_ = smgr;
}

Ogre::SceneManager* Line3D::GetSmgr() const
{
	return Smgr_;
}

void Line3D::Destory()
{
	detachFromParent();

	Smgr_->destroyMovableObject(this);
}

void Line3D::SetColor(const Ogre::ColourValue& val)
{
	Ogre::Vector4 clr;
	clr.x = val.r;
	clr.y = val.g;
	clr.z = val.b;
	clr.w = val.a;
	setCustomParameter(0, clr);
}

Ogre::ColourValue Line3D::GetColor() const
{
	auto val = getCustomParameter(0);
	return Ogre::ColourValue(val.x, val.y, val.z, val.w);
}

void Line3D::SetLineWidth(float val)
{
	Width_ = val;

	_UpdateTransform();
}

float Line3D::GetLineWidth() const
{
	return Width_;
}

void Line3D::SetStartPoint(const Ogre::Vector3& pos)
{
	SetPoint(pos, EndPnt_);

	_UpdateTransform();
}

const Ogre::Vector3& Line3D::GetStartPoint() const
{
	return StartPnt_;
}

void Line3D::SetEndPoint(const Ogre::Vector3& pos)
{
	SetPoint(StartPnt_, pos);

	_UpdateTransform();
}

const Ogre::Vector3& Line3D::GetEndPoint() const
{
	return EndPnt_;
}

Ogre::Vector3 Line3D::GetCenter() const
{
	return (StartPnt_ + EndPnt_) / 2;
}

void Line3D::SetPoint(const Ogre::Vector3& start, const Ogre::Vector3& end)
{
	StartPnt_ = start;
	EndPnt_ = end;
	LineDir_ = (EndPnt_ - StartPnt_).normalisedCopy();

	_UpdateTransform();
}

const Ogre::Vector3& Line3D::GetLineDir() const
{
	return LineDir_;
}

void Line3D::_UpdateTransform()
{
	auto vec = EndPnt_ - StartPnt_;
	auto norVec = vec;
	norVec.normalise();

	LocalMat_.makeTransform((EndPnt_ + StartPnt_) / 2,
							Ogre::Vector3(vec.length(), Width_, Width_),
							Ogre::Vector3::UNIT_X.getRotationTo(norVec));


	Ogre::Aabb aabb(AABB_.getCenter(), AABB_.getHalfSize());
	aabb.transformAffine(LocalMat_);
	mObjectData.mLocalAabb->setFromAabb(aabb, mObjectData.mIndex);
	mObjectData.mLocalRadius[mObjectData.mIndex] = aabb.getRadius();
}