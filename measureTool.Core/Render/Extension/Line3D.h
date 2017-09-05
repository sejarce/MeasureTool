#pragma once

#include "OgreMovableObject.h"
#include "OgreRenderable.h"
#include "OgrePrerequisites.h"
#include "OgreRenderOperation.h"

#include "TExtMovableObjFactory.h"

class	Line3D : public Ogre::MovableObject, public Ogre::Renderable
{
	Ogre::RenderOperation	RO_;
	Ogre::AxisAlignedBox	AABB_;
	Ogre::MaterialPtr		Material_;
	Ogre::Matrix4			LocalMat_;
	Ogre::Vector3			StartPnt_;
	Ogre::Vector3			EndPnt_;
	Ogre::Vector3			LineDir_;
	float					Width_;
	Ogre::SceneManager*		Smgr_ = {};

public:

	Line3D(const Ogre::IdType& id, Ogre::ObjectMemoryManager *objectMemoryManager);

	virtual ~Line3D();

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

	void	SetLineWidth(float val);

	float	GetLineWidth() const;

	void	SetStartPoint(const Ogre::Vector3& pos);

	const Ogre::Vector3& GetStartPoint() const;

	void	SetEndPoint(const Ogre::Vector3& pos);

	const Ogre::Vector3& GetEndPoint() const;

	void	SetPoint(const Ogre::Vector3& start, const Ogre::Vector3& end);

	Ogre::Vector3	GetCenter() const;

	const Ogre::Vector3&	GetLineDir() const;

	void	SetSmgr(Ogre::SceneManager* smgr);

	Ogre::SceneManager*	GetSmgr() const;

	void	Destory();

private:

	void	_UpdateTransform();
};

class	Line3DFactory : public TExtMovableObjFactory<Line3D>
{
public:
};