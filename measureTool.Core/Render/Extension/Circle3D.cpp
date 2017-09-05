#include "Circle3D.h"

#include "Render/RenderGroupID.h"
#include "Render/QueryMask.h"
#include "Render/PrefabResourceMgr.h"

#include "Ogre.h"

Circle3D::Circle3D(const Ogre::IdType& id, Ogre::ObjectMemoryManager *objectMemoryManager)
	:MovableObject(id, objectMemoryManager, ERG_Manipulator_RingFace)
{
	auto mesh = PrefabResourceMgr::GetInstance().GetMesh(PrefabResourceMgr::EPM_UnitCircle);
	mesh->load();

	decltype( RO_ ) meshRO;
	mesh->getSubMesh(0)->_getRenderOperation(meshRO);

	RO_.operationType = Ogre::RenderOperation::OT_TRIANGLE_FAN;
	RO_.vertexData = meshRO.vertexData->clone(false);
	RO_.useIndexes = false;

	AABB_ = mesh->getBounds();

	Material_ = PrefabResourceMgr::GetInstance().GetMateiral(PrefabResourceMgr::EPMAT_Manipulator_CircleFace);

	setCustomParameter(0, Ogre::Vector4(Ogre::Vector3::UNIT_SCALE));
}

Circle3D::~Circle3D()
{
	delete RO_.vertexData;
}

const Ogre::String& Circle3D::getMovableType(void) const
{
	return Circle3DFactory::GetFactoryName();
}

const Ogre::LightList& Circle3D::getLights(void) const
{
	return queryLights();
}

void Circle3D::getRenderOperation(Ogre::RenderOperation& op)
{
	op = RO_;
}

void Circle3D::getWorldTransforms(Ogre::Matrix4* xform) const
{	
	*xform = getParentNode()->_getFullTransform();
}

void Circle3D::_updateRenderQueue(Ogre::RenderQueue* queue, Ogre::Camera *camera, const Ogre::Camera *lodCamera)
{
	if ( RO_.vertexData->vertexCount > 2 )
	{
		queue->addRenderable(this, mRenderQueueID, mRenderQueuePriority);
	}
}

void Circle3D::visitRenderables(Renderable::Visitor* visitor, bool debugRenderables /*= false*/)
{
	visitor->visit(this, 0, false);
}

const Ogre::MaterialPtr& Circle3D::getMaterial(void) const
{
	return Material_;
}

Ogre::Real Circle3D::getSquaredViewDepth(const Ogre::Camera* cam) const
{
	auto diff = (getParentNode()->_getFullTransform()).getTrans() - cam->getDerivedPosition();
	return diff.squaredLength();
}

const Ogre::UserObjectBindings& Circle3D::GetUserObjectBindings() const
{
	return Ogre::MovableObject::getUserObjectBindings();
}

Ogre::UserObjectBindings& Circle3D::GetUserObjectBindings()
{
	return Ogre::MovableObject::getUserObjectBindings();
}

void Circle3D::SetSmgr(Ogre::SceneManager* smgr)
{
	Smgr_ = smgr;
}

Ogre::SceneManager* Circle3D::GetSmgr() const
{
	return Smgr_;
}

void Circle3D::SetColor(const Ogre::ColourValue& val)
{
	Ogre::Vector4 clr;
	clr.x = val.r;
	clr.y = val.g;
	clr.z = val.b;
	clr.w = val.a;
	setCustomParameter(0, clr);
}

Ogre::ColourValue Circle3D::GetColor() const
{
	auto val = getCustomParameter(0);
	return Ogre::ColourValue(val.x, val.y, val.z, val.w);
}

void Circle3D::SetDegree(uint32_t deg)
{
	deg = deg % 361;

	RO_.vertexData->vertexCount = 2 + deg;
}

void Circle3D::Destory()
{
	detachFromParent();

	Smgr_->destroyMovableObject(this);
}