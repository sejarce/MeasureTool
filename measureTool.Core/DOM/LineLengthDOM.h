#pragma once

#include "IDOM.h"

#include "OgrePrerequisites.h"

class	LineLengthDOM : public IDOM
{
	class	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

public:

	LineLengthDOM(const DocumentSPtr& doc,uint32_t index);

	~LineLengthDOM();

public:

	virtual EDOMType GetType() const override;

public:

	void	SetPoints(const Ogre::Vector3& p1, const Ogre::Vector3& p2);

	const Ogre::Vector3&	GetPoint1() const;

	const Ogre::Vector3&	GetPoint2() const;

protected:

	virtual void OnUpdate(float& val, PntList& pntList) override;

	virtual void OnDeserialize(const std::string& buf) override;

	virtual void OnDeserialize(const PntList& pList) override;

};