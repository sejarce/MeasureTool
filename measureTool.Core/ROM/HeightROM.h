#pragma once

#include "IROM.h"

#include "Render/Extension/Line3DFwd.h"
#include "Render/Extension/Point3DFwd.h"

class	HeightROM : public TROM<HeightROM>
{
	class	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

public:

	HeightROM(Ogre::SceneNode* rootNode, const IDOMSPtr& dom);

	~HeightROM();

public:

	const Ogre::Vector3&  GetStartPoint() const;

	const Ogre::Vector3&  GetEndPoint() const;

	void	SetPoint(const Ogre::Vector3& sp, const Ogre::Vector3& ep);

protected:

	virtual void OnSetVisible(bool val) override;

	virtual void OnSetPickingState(EPickingState state) override;

	virtual	void OnSetDisplayMode(EDisplayMode mode) override;

	virtual boost::optional<float> OnGetDistanceToRay(const TopoDS_Edge& ray, float deviation) override;

	virtual void OnStashSave() override;

	virtual void OnStashPop() override;
};