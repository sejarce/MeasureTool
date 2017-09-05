#pragma once

#include "OgreMovableObject.h"
#include "OgrePrerequisites.h"

class	IExtMovableObjFactory : public Ogre::MovableObjectFactory
{
public:

	virtual	void	Init()
	{}

	virtual	void	UnInit()
	{}
};

template<typename MovableT>
class TExtMovableObjFactory : public IExtMovableObjFactory
{
public:

	TExtMovableObjFactory() {}

	~TExtMovableObjFactory() {}

public:

	virtual Ogre::MovableObject* createInstanceImpl(Ogre::IdType id, Ogre::ObjectMemoryManager *objectMemoryManager, const Ogre::NameValuePairList* params = 0) override
	{
		return new MovableT(id, objectMemoryManager);
	}

	virtual const Ogre::String& getType(void) const override
	{
		return GetFactoryName();
	}

	virtual void destroyInstance(Ogre::MovableObject* obj) override
	{
		delete obj;
	}

public:

	static const	std::string&	GetFactoryName()
	{
		static std::string factoryName = typeid( MovableT ).name();

		return factoryName;
	}

	static	MovableT*	CreateInstance(Ogre::SceneManager* smgr, Ogre::SceneMemoryMgrTypes type = Ogre::SCENE_DYNAMIC)
	{
		auto ret = static_cast<MovableT*>( smgr->createMovableObject(GetFactoryName(), &( smgr->_getEntityMemoryManager(type) )) );
		ret->SetSmgr(smgr);

		return ret;
	}
};