#pragma once

#include "IDOM.h"

#include "OgrePrerequisites.h"

class	TopoDS_Edge;

class	CurveLengthDOM : public IDOM
{
	class	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;
public:
	using Pnt3D = std::pair<int, Ogre::Vector3>;
	using Pnt3DList = std::vector<Pnt3D>;

public:

	CurveLengthDOM(const DocumentSPtr& doc, uint32_t index);

	~CurveLengthDOM();

public:

	virtual EDOMType GetType() const override;

public:
	const Pnt3D&	GetStartPos() const;

	const Pnt3D&	GetEndPos() const;

	TopoDS_Edge				GetEdge() const;

	void	UpdateCurve(const Pnt3D& startpos, const Pnt3D& endpos, const PntList& pnlList);

protected:

	virtual void OnUpdate(float& val, PntList& pntList) override;

	virtual void OnDeserialize(const std::string& buf) override;

	virtual void OnDeserialize(const PntList& pList) override;

	virtual void OnSerialize(std::string& buf) const override;
};