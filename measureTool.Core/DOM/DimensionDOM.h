#pragma once

#include "IDOM.h"

#include "OgrePrerequisites.h"

class	TopoDS_Edge;

class	DimensionDOM : public IDOM
{
	class	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

public:

	DimensionDOM(const DocumentSPtr& doc, uint32_t index, EDOMType domType);

	~DimensionDOM();

public:

	virtual EDOMType GetType() const override;

public:

	const Ogre::Vector3&	GetPlnNormal() const;

	const Ogre::Vector3&	GetPlnPos() const;

	TopoDS_Edge				GetEdge() const;

	void	UpdateCurve(const Ogre::Vector3& plnNormal, const PntList& pnlList);

protected:

	virtual void OnUpdate(float& val, PntList& pntList) override;

	virtual void OnDeserialize(const std::string& buf) override;

	virtual void OnDeserialize(const PntList& pList) override;

	virtual void OnSerialize(std::string& buf) const override;
};