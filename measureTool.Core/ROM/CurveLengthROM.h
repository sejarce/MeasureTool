#pragma once

#include "IROM.h"

class	CurveLengthROM : public TROM<CurveLengthROM>
{
	class	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

public:
	using Pnt3D = std::pair<int, Ogre::Vector3>;

public:

	CurveLengthROM(Ogre::SceneNode* rootNode, const IDOMSPtr& dom);

	~CurveLengthROM();

public:
	void Update(const Pnt3D& startpos,
		const Pnt3D& endpos,
		const std::vector<Ogre::Vector3>& plnList);

	const std::vector<Ogre::Vector3>&	GetFittingList() const;

	const Pnt3D&	GetStartPos() const;

	const Pnt3D&	GetEndPos() const;

	void	SaveToFile(const std::wstring& fileName);

protected:

	virtual void OnSetVisible(bool val) override;

	virtual void OnSetPickingState(EPickingState state) override;

	virtual	void OnSetDisplayMode(EDisplayMode mode) override;

	virtual boost::optional<float> OnGetDistanceToRay(const TopoDS_Edge& ray, float deviation) override;

	virtual void OnStashSave() override;

	virtual void OnStashPop() override;
};