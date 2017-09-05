#pragma once

#include "OgreMovableObject.h"
#include "OgreRenderable.h"
#include "OgrePrerequisites.h"
#include "OgreRenderOperation.h"

#include "TExtMovableObjFactory.h"

class	Point3D : public Ogre::MovableObject, public Ogre::Renderable
{
	Ogre::RenderOperation	RO_;
	Ogre::AxisAlignedBox	AABB_;
	Ogre::MaterialPtr		Material_;
	Ogre::Matrix4			LocalMat_;
	Ogre::Vector3			Pnt_;
	float					Radius_;
	Ogre::SceneManager*		Smgr_ = {};

public:

	Point3D(const Ogre::IdType& id, Ogre::ObjectMemoryManager *objectMemoryManager);

	virtual ~Point3D();

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

	void	SetColor(const Ogre::ColourValue& val);

	Ogre::ColourValue	GetColor() const;

	const Ogre::UserObjectBindings& GetUserObjectBindings() const;

	Ogre::UserObjectBindings& GetUserObjectBindings();

	void	SetRadius(float val);

	float	GetRadius() const;

	const Ogre::Vector3& GetPoint() const;

	void	SetPoint(const Ogre::Vector3& postion);

	void	SetSmgr(Ogre::SceneManager* smgr);

	Ogre::SceneManager*	GetSmgr() const;

	void	Destory();

private:

	void	_UpdateTransform();
};

class	Point3DFactory : public TExtMovableObjFactory<Point3D>
{
};