#pragma once

#include "IROM.h"

class	DimensionROM : public TROM<DimensionROM>
{
	class	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

public:

	DimensionROM(Ogre::SceneNode* rootNode, const IDOMSPtr& dom);

	~DimensionROM();

public:

	void	Update(const std::vector<Ogre::Vector3>& plnList, const Ogre::Vector3& plnNormal);

	const std::vector<Ogre::Vector3>&	GetFittingList() const;

	const Ogre::Vector3&	GetCenter() const;

	const Ogre::Vector3&	GetNormal() const;

	std::tuple<float, float>	GetRotation() const;

	void	SetRotation(float xdeg, float zdeg);

	void	SaveToFile(const std::wstring& fileName);

protected:

	virtual void OnSetVisible(bool val) override;

	virtual void OnSetPickingState(EPickingState state) override;

	virtual	void OnSetDisplayMode(EDisplayMode mode) override;

	virtual boost::optional<float> OnGetDistanceToRay(const TopoDS_Edge& ray, float deviation) override;

	virtual void OnStashSave() override;

	virtual void OnStashPop() override;
};