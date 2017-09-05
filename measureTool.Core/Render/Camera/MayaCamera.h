#pragma once

#include "Render/ICameraFrameListener.h"

#include "OgrePrerequisites.h"
#include "OgreVector3.h"

#include <memory>

class	MayaCamera : public ICameraFrameListener
{
	class	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

public:

	MayaCamera(Ogre::Camera* camera, Ogre::RenderTarget* rt);

	~MayaCamera();

private:

	virtual void	_FrameStart(const Ogre::FrameEvent& fevt) override ;

	virtual void	ReAttach() override ;

public:

	void SetPosition(const Ogre::Vector3& pos);

	Ogre::Vector3	GetPosition() const;

	void SetTarget(const Ogre::Vector3& target);

	Ogre::Vector3	GetTarget() const;

	Ogre::Vector3	GetDir() const;

	void	SetPosAndTarget(const Ogre::Vector3& pos, const Ogre::Vector3& target);

	void	SetFocusLength(float length);

	float	GetFocusLength() const;

	void	SetRotationSpeed( float rSpeed);

	float GetRotationSpeed() const;
};

using	MayaCameraSPtr = std::shared_ptr<MayaCamera>;