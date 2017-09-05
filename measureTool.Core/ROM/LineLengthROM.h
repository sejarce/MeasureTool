#pragma once

#include "IROM.h"

#include "Render/Extension/Line3DFwd.h"
#include "Render/Extension/Point3DFwd.h"

class	LineLengthROM : public TROM<LineLengthROM>
{
	class	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

public:

	LineLengthROM(Ogre::SceneNode* rootNode, const IDOMSPtr& dom);

	~LineLengthROM();

public:

	const Ogre::Vector3&  GetStartPoint() const;

	const Ogre::Vector3&  GetEndPoint() const;

	void	SetPoint(const Ogre::Vector3& start, const Ogre::Vector3& end);

protected:

	virtual void OnSetVisible(bool val) override;

	virtual void OnSetPickingState(EPickingState state) override;

	virtual	void	OnSetDisplayMode(EDisplayMode mode) override;

	virtual boost::optional<float> OnGetDistanceToRay(const TopoDS_Edge& ray, float deviation) override;

	virtual void OnStashSave() override;

	virtual void OnStashPop() override;
};