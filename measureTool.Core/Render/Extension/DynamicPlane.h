#pragma once

#include "OgreMovableObject.h"
#include "OgreRenderable.h"
#include "OgrePrerequisites.h"
#include "OgreRenderOperation.h"

#include "TExtMovableObjFactory.h"

class	DynamicPlane : public Ogre::MovableObject, public Ogre::Renderable
{
	Ogre::RenderOperation	RO_;
	Ogre::AxisAlignedBox	AABB_;
	Ogre::MaterialPtr		Material_;
	Ogre::SceneManager*		Smgr_ = {};

public:

	DynamicPlane(const Ogre::IdType& id, Ogre::ObjectMemoryManager *objectMemoryManager);

	virtual ~DynamicPlane();

public://Overrides from MovableObject

	//Dummy
	virtual const Ogre::String& getMovableType(void) const override;

	virtual void _updateRenderQueue(Ogre::RenderQueue* queue, Ogre::Camera *camera, const Ogre::Camera *lodCamera) override;

	virtual void visitRenderables(Renderable::Visitor* visitor, bool debugRenderables = false) override;

public://Overrides from Renderable

	//Dummy
	virtual const Ogre::LightList& getLights(void) const override;

	virtual void getRenderOperation(Ogre::RenderOperation& op) override;

	virtual void getWorldTransforms(Ogre::Matrix4* xform) const override;

	virtual const Ogre::MaterialPtr& getMaterial(void) const override;

	virtual Ogre::Real getSquaredViewDepth(const Ogre::Camera* cam) const override;

public:

	const Ogre::UserObjectBindings& GetUserObjectBindings() const;

	Ogre::UserObjectBindings& GetUserObjectBindings();

	void	SetSmgr(Ogre::SceneManager* smgr);

	Ogre::SceneManager*	GetSmgr() const;

	void	SetColor(const Ogre::ColourValue& clr);

	void	SetBound(const Ogre::AxisAlignedBox& box);

	void	UpdatePln(const std::vector<Ogre::Vector3>& plnList);

	void	Destory();
};

class	DynamicPlaneFactory : public TExtMovableObjFactory<DynamicPlane>
{
public:
};