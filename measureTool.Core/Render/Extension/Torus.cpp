#include "Torus.h"

#include "Render/RenderGroupID.h"
#include "Render/QueryMask.h"
#include "Render/PrefabResourceMgr.h"

#include "Ogre.h"

Torus::Torus(const Ogre::IdType& id, Ogre::ObjectMemoryManager *objectMemoryManager)
	:MovableObject(id, objectMemoryManager, ERG_Manipulator_Ring)
{
	auto mesh = PrefabResourceMgr::GetInstance().GetMesh(PrefabResourceMgr::EPM_UnitTorus);
	mesh->load();

	mesh->getSubMesh(0)->_getRenderOperation(RO_);

	AABB_ = mesh->getBounds();

	Material_ = PrefabResourceMgr::GetInstance().GetMateiral(PrefabResourceMgr::EPMAT_Manipulator_Torus);

	setCustomParameter(0, Ogre::Vector4(Ogre::Vector3::UNIT_SCALE));
}

Torus::~Torus()
{
	
}

const Ogre::String& Torus::getMovableType(void) const
{
	return TorusFactory::GetFactoryName();
}

const Ogre::LightList& Torus::getLights(void) const
{
	return queryLights();
}

void Torus::getRenderOperation(Ogre::RenderOperation& op)
{
	op = RO_;
}

void Torus::getWorldTransforms(Ogre::Matrix4* xform) const
{	
	*xform = getParentNode()->_getFullTransform();
}

void Torus::_updateRenderQueue(Ogre::RenderQueue* queue, Ogre::Camera *camera, const Ogre::Camera *lodCamera)
{
	queue->addRenderable(this, mRenderQueueID, mRenderQueuePriority);
}

void Torus::visitRenderables(Renderable::Visitor* visitor, bool debugRenderables /*= false*/)
{
	visitor->visit(this, 0, false);
}

const Ogre::MaterialPtr& Torus::getMaterial(void) const
{
	return Material_;
}

Ogre::Real Torus::getSquaredViewDepth(const Ogre::Camera* cam) const
{
	auto diff = (getParentNode()->_getFullTransform()).getTrans() - cam->getDerivedPosition();
	return diff.squaredLength();
}

const Ogre::UserObjectBindings& Torus::GetUserObjectBindings() const
{
	return Ogre::MovableObject::getUserObjectBindings();
}

Ogre::UserObjectBindings& Torus::GetUserObjectBindings()
{
	return Ogre::MovableObject::getUserObjectBindings();
}

void Torus::SetSmgr(Ogre::SceneManager* smgr)
{
	Smgr_ = smgr;
}

Ogre::SceneManager* Torus::GetSmgr() const
{
	return Smgr_;
}

void Torus::SetColor(const Ogre::ColourValue& val)
{
	Ogre::Vector4 clr;
	clr.x = val.r;
	clr.y = val.g;
	clr.z = val.b;
	clr.w = val.a;
	setCustomParameter(0, clr);
}

Ogre::ColourValue Torus::GetColor() const
{
	auto val = getCustomParameter(0);
	return Ogre::ColourValue(val.x, val.y, val.z, val.w);
}

void Torus::Destory()
{
	detachFromParent();

	Smgr_->destroyMovableObject(this);
}

void TorusFactory::Init()
{
	
}

void TorusFactory::UnInit()
{

}
